date 
echo "Usage: source <expid> <runid> <location> <duration>  "

# =========================== Starting ROS crawler bag ========================

#source /opt/ros/humble/setup.bash
#. ~/ros2_ws/install/setup.bash

#ros2 bag play ~/bagfiles/crawler --loop

# =============================================================================


echo "testing AX 20 MHz @ 6  GHz"

#nmcli dev wifi con "INSA_ASUS_6G" password "insagora2022"
#wait

echo "start"
source run.bash $1"_ax_20mhz_6ghz" $2 $3 $4
echo "end"

echo "test finished"


