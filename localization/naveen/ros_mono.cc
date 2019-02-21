/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<nav_msgs/Odometry.h> //
#include<geometry_msgs/Point.h> //
#include<pangolin/pangolin.h>
#include<std_msgs/UInt8.h> //
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

enum ControlSignal {
    Empty = 0,
    RecordPosition = 1,
    Rescale = 2,
    Recenter = 3
};

class Control 
{
    public:
        static ControlSignal signal;
        static std::vector<cv::Mat> points;
        static float computeScaleFactor();
        Control();
        void callback(std_msgs::UInt8 control);


};

ros::Publisher *g_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    //----QuadcpterAR--
    ros::Publisher pub = nodeHandler.advertise<nav_msgs::Odometry>("MonoPose", 1);
    g_pub = &pub;
    // -----------------
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    
    Control control;
    ros::Subscriber subControl = nodeHandler.subscribe<std_msgs::UInt8>("/orb_slam2/control", 1, &Control::callback, &control);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void rotationMatrixToQuaternion(pangolin::OpenGlMatrix &M, float &qw, float &qx, float &qy, float &qz) {
    float 
        m00 = M.m[0],
        m10 = M.m[1],
        m20 = M.m[2],

        m01 = M.m[4],
        m11 = M.m[5],
        m21 = M.m[6],

        m02 = M.m[8],
        m12 = M.m[9],
        m22 = M.m[10];

    qw = sqrt((1 + m00 + m11 + m22))/2;
    qx = (m21 - m12)/( 4 * qw);
    qy = (m02 - m20)/( 4 * qw);
    qz = (m10 - m01)/( 4 * qw);
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    /// By QuadcopterAR.
    //  Publish camera position as a ROS msg.
    pangolin::OpenGlMatrix M;
    mpSLAM->mpMapDrawer->GetCurrentOpenGLCameraMatrix(M);

    if(g_pub != NULL) {
        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map_fid";
        msg.child_frame_id = "base_link_fid";
        msg.pose.pose.position.y = 0-M.m[12]; //tx
        msg.pose.pose.position.z = 0-M.m[13]; //ty
        msg.pose.pose.position.x = M.m[14]; //tz

        // Orientation in quaternion.
        float qw, qx, qy, qz;
        rotationMatrixToQuaternion(M, qw, qx, qy, qz);

        msg.pose.pose.orientation.w = qw;
        msg.pose.pose.orientation.y = 0.0-qx;
        msg.pose.pose.orientation.z = 0.0-qy;
        msg.pose.pose.orientation.x = qz;

        g_pub->publish(msg);
        
        if(Control::signal == RecordPosition){
                cv::Mat position = (cv::Mat_<double>(3, 1) << M.m[12], M.m[13], M.m[14]);

                Control::points.push_back(position);
                std::cout << "recorded position\n" << position << std::endl;
                Control::signal = Empty;
        }
                
        //  else if(Control::signal == Rescale) {
        //         float scaleFactor = Control::computeScaleFactor();
        //         std::cout << "Rescaling...\n" << "scaleFactor: " << scaleFactor << std::endl;

        //         mpSLAM->Rescale(scaleFactor);
        //         Control::points.clear(); //
        //         Control::signal = Empty;
        // } else if(Control::signal == Recenter) {
        //         std::cout << "Recentering.\n";
        //         mpSLAM->Recenter();
        //         std::cout << "Recentered.\n";
        //         Control::signal = Empty;
        // }
    }
}


ControlSignal Control::signal = Empty;
std::vector<cv::Mat> Control::points;
Control::Control() {

}

void Control::callback(std_msgs::UInt8 control) {
    Control::signal = (ControlSignal)(control.data);
    std::cout << "received signal: " << Control::signal  << std::endl;
}

float Control::computeScaleFactor(){
    float physicalLength[3] = {59.325, 52.35, 51.975};
    float virtualLength[3];

    if(Control::points.size() < 3)
        return -1;

    for(int i = 0; i < 2; i++) {
        virtualLength[i] = cv::norm(Control::points[i], Control::points[i+1]) ;
    }

    virtualLength[2] = cv::norm(Control::points[2], Control::points[0]) ;

    float scaleFactor = 0, s = 0;
    for(int i = 0; i < 3; i++) {
        s = physicalLength[i] / virtualLength[i];

        std::cout << "virtual length " << i << " :" << virtualLength[i] << std::endl;
        std::cout << "scale factor " << i << " :" << s << std::endl;
        scaleFactor += s;
    }

    return scaleFactor / 3.0;
}