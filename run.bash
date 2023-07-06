#source ~/catkin_ws/start.bash

export this_expid=$1
export runid=$2
export loc=$3
export duration=$4
export bw=1000M

source ros2_start.bash 

echo ">>" $this_expid,$runid,$loc, $duration, $bw

#=========================== Iperf test =======================================
echo "starting full test.. duration:"$3 "s"

date +"%T"
python3 cmd_logger.py $this_expid"_tcp_single_UL" $runid $loc $duration ap_scan "iw dev wlp0s20f3 scan" 0.033 & #rate once very 30 sec
python3 cmd_logger.py $this_expid"_tcp_single_UL" $runid $loc $duration ptp_status "cat /var/run/ptpd2.status" 2 &
python3 cmd_logger.py $this_expid"_tcp_single_UL" $runid $loc $duration phy "iw dev wlp0s20f3 link" 2 &
python3 iperf.py $this_expid $runid"_tcp_single_UL_"$loc "iperf3 -c 192.168.50.155 -p 3000  -J -t "$duration" -b "$bw &
ros2 run tx_control listener $this_expid $runid $loc "robot_control" $duration

echo "finished.."

wait

