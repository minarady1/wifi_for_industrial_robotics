# Wi-Fi evaluation for industrial robotics   
This the source code and the source data obtained for our experiments in evaluating WiFi perfrance for industrial robotics in the Perama Shipyard, Piraeus, Greece. 

## raw data
The collected raw data are available under the [logs](logs) directory. 

## post-processed clean database
The raw data are cleaned and organized in one [JSON Database](plots/perama_range_testing.json)
The JSON file structured as follows:

* Locations (4 locations)
    * PHY configuration (9 configurations for each location)
        * Parameter (5 groups of parameters with group prefix, timestap array for each group)
            * prefixes: control (from ros application) , ptp (from PTP deamon), phy (network interface PHY stats), apscan (access point scans), iperf (logged iperf metrics) 
            * Array of measurements (one per parameter) 
        
 
## Scripts

- WiFi configurations are grouped in different `main_x.bash` files.
- The script in each `main_x.bash` connects to a certain frequencey band then executes the `run.bash` script which intiates the measurement campaign
- `run.bash` script starts the following scripts in parallel:
  * iPerf script for throughput testing using simple custom-made `iperf.py` script
  * ROS control subscriber for control packet reception and logging over ROS2
  * PTP status capture using a simple custom-made `cmd_logger.py` tool
  * Network Interface statistics capture using `cmd_logger`
  * Access Point scanning using `cmd_logger`
 
For any questions, feel free to reach me at mina.rady@insa-lyon.fr 
