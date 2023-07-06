#! /bin/sh
set -x

python iperf_plotter.py loc1_ax_20mhz_24ghz run5 
python iperf_plotter.py loc1_ax_20mhz_5ghz run5 
python iperf_plotter.py loc1_ax_80mhz_5ghz run5 
python iperf_plotter.py loc1_ac_20mhz_5ghz run5 
python iperf_plotter.py loc1_ac_80mhz_5ghz run5 
python iperf_plotter.py loc1_n_20mhz_24ghz run5 
python iperf_plotter.py loc1_n_20mhz_5ghz run5  
python iperf_plotter.py loc1_g_20mhz_24ghz run5 
python iperf_plotter.py loc1_a_20mhz_24ghz run5 

$SHELL