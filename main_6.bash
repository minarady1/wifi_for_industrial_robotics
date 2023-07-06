date 
echo "Usage: source <expid> <runid> <location> <duration>  "

# =========================== Starting ROS crawler bag ========================

#source /opt/ros/humble/setup.bash
#. ~/ros2_ws/install/setup.bash

#ros2 bag play ~/bagfiles/crawler --loop

# =============================================================================

echo "testing AC 20 MHz @ 5  GHz"

nmcli dev wifi con "INSA_ASUS_5G" password "insagora2022"
wait

echo "start"
source run.bash $1"_ac_20mhz_5ghz" $2 $3 $4
echo "end"


wait