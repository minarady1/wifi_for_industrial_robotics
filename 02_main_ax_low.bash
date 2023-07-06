date 
#input: 1.epxid 2.rundid 3.loc 4.duration

bw=1000M #mbps

export expid=$1
export runid=$2
export loc=$3
export duration=$4

echo "###" $expid,$runid,$loc, $duration, $bw

# =============================================================================

# echo "testing AX 20 MHz @ 6  GHz"

nmcli dev wifi con "INSA_ASUS_6G" password "insagora2022"
wait

echo "start"
source run.bash $expid"_ax_20mhz_6ghz"  $runid $loc $duration $bw
echo "end"

echo "test finished"

# =============================================================================

# echo "testing AX 20 MHz @ 5  GHz"

nmcli dev wifi con "INSA_ASUS_5G" password "insagora2022"
wait

echo "start"
source run.bash $expid"_ax_20mhz_5ghz"  $runid $loc $duration $bw
echo "end"

wait



# =============================================================================


# echo "testing AX 20 MHz @ 24  GHz"

nmcli dev wifi con "INSA_ASUS" password "insagora2022"
sleep 5
wait

echo "start"
source run.bash $expid"_ax_20mhz_24ghz" $runid $loc $duration $bw
echo "end"

wait



