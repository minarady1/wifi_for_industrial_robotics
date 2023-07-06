date 
echo "Usage: source <expid> <runid> <location> <duration>  "

# =========================== Starting ROS crawler bag ========================

#source /opt/ros/humble/setup.bash
#. ~/ros2_ws/install/setup.bash

#ros2 bag play ~/bagfiles/crawler --loop

# =============================================================================

echo "testing AX 80 MHz @ 5  GHz"

nmcli dev wifi con "INSA_ASUS_5G" password "insagora2022"
wait

echo "start"
source run.bash $1"_ax_80mhz_5ghz" $2 $3 $4
echo "end"

wait

# =============================================================================

echo "testing AX 20 MHz @ 24  GHz"

nmcli dev wifi con "INSA_ASUS" password "insagora2022"
wait

echo "start"
source run.bash $1"_ax_20mhz_24ghz" $2 $3 $4
echo "end"

wait