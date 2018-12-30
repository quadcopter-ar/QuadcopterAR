- To simulate no gps availability to mimic our project conditions, set the `SIM_GPS_DISABLE` parameter to 1. 

  - In MavProxy shell:

    `param set SIM_GPS_DISABLE 1` 

  - Can also use QGroundControl parameter menu

- Disable all pre-arm checks in QGC

- Set roll rate D gain to a low number (.0001) to avoid unstable behavior in simulation

  - In MavProxy shell:

    `param set ATC_RAT_RLL_D 0.000100`

- Don't use the compasses:

  - `COMPASS_USE1` = 0
  - `COMPASS_USE2` = 0
  - `COMPASS_USE3` = 0

- Other parameters:

  - `FS_EKF_THRESH`= 1
  - `AHRS_EKF_TYPE` = 2 
  - `EKF2_ENABLE` = 1 
  - `EKF3_ENABLE `= 0 
  - `GPS_TYPE` = 0 
  - `EK2_GPS_TYPE` = 3 
  - `VISO_TYPE` = 0