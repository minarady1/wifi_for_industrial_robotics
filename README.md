# wifi_for_industrial_robotics
This the source code and the source data obtained for our experiments in evaluating WiFi perfrance for industrial robotics

## raw data
The collected raw data are available under the `/logs' directory. 

## post-processed clean database
The raw data are cleaned and organized in one [JSON Database](plots/perama_range_testing.json)
The JSON file structured as follows:

Locations (4 locations)
    PHY configuration (9 configurations for each location)
      Parameter (4 groups of parameters, timestap array for each group)
        array of measurements (one per parameter) 
        
 
