date 

# =============================================================================

# echo "testing N 20 MHz @ 24  GHz"

nmcli dev wifi con "INSA_ASUS" password "insagora2022"
wait

echo "start"
source run.bash $1"_n_20mhz_24ghz" $2 $3 $4
echo "end"

wait

# =============================================================================

# echo "testing n 20 MHz @ 5  GHz"

nmcli dev wifi con "INSA_ASUS_5G" password "insagora2022"
wait

echo "start"
source run.bash $1"_n_20mhz_5ghz" $2 $3 $4
echo "end"

wait

# =============================================================================

# echo "testing AX 80 MHz @ 6  GHz"

nmcli dev wifi con "INSA_ASUS_6G" password "insagora2022"
wait

echo "start"
source run.bash $1"_ax_80mhz_6ghz" $2 $3 $4
echo "end"

echo "test finished"


