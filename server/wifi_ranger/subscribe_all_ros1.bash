#!/usr/bin/env bash
rostopic echo /camera/color/camera_info&
rostopic echo /camera/color/image_raw&
rostopic echo /clock&
rostopic echo /cmd_vel&
rostopic echo /filtered_imu&
rostopic echo /imu/imu&
rostopic echo /joy&
rostopic echo /leica/leica_point&
rostopic echo /leica/position&
rostopic echo /livox/lidar&
rostopic echo /m1_raw&
rostopic echo /m1m2_enc_accum&
rostopic echo /m2_raw&
rostopic echo /odom&
rostopic echo /odom_raw&
rostopic echo /robot_control&
rostopic echo /rosout&
rostopic echo /rosout_agg&
rostopic echo /tf&
rostopic echo /tf_static&
rostopic echo /uwb_driver_node/uwb&

