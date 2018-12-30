#!/bin/bash

export ROS_IP=$(hostname -I)
echo "ROS_IP=$ROS_IP"
read -p "MASTER_IP=" MASTER_IP
export ROS_MASTER_URI=http://${MASTER_IP}:11311
echo "ROS_MASTER_URI=$ROS_MASTER_URI"