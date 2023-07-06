# -*- coding: utf-8 -*-
'''
Pre-processor v 1.0

Preprocseeing script for captured measurements

Input: configuration labels, log files
Outpute: json file containing time series logs per metric per configuration
    per location


Bugwright2 Project
Mina Rady mina.rady@insa-lyon.fr

'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

import json
import sys
import pdb
import pandas as pd
import time
import re
import traceback
import copy

#=========================== Declarations ======================================

# expid, runid, duration, number of locations


expid       = "perama_site"
locations   = 4

# all configs use run1 execpt the following use run2
# list of 80211 configs

configs = [
    "ax_20mhz_5ghz"  ,   # run2
    "ax_20mhz_24ghz" ,   # run2
    "ax_80mhz_5ghz"  ,   # run2               
    "ax_160mhz_6ghz" ,   
    "ax_80mhz_6ghz"  ,
    "ax_20mhz_6ghz"  ,
    "ac_80mhz_5ghz"  ,
    "ac_20mhz_5ghz"  ,
    "n_20mhz_24ghz"  ,
]

runs = [
    "run2",
    "run2",
    "run2",
    "run1",
    "run1",
    "run1",
    "run1",
    "run1",
    "run1",
]


#============================ Global variables =================================

# list of log file vars

data            = {}
prev_seqnum     = -1
reference_time  = -1

#============================ Functions ========================================

def initialize ():
    global data
    l = 1
    while l <= locations:
        data [l] = {}
        for c in configs:
             data [l][c]={}
        l=l+1

#---------------------------- Row parsers --------------------------------------

# parser of control logs
# input: text row
# output list: dict [var] = [time] [val]

def parse_control_log_line (line):
    global prev_seqnum
    global reference_time
    loss = 0
    
    obj = json.loads(line)
    delay_ms = (obj ["timestamp"] - int(obj ["payload"]["src_timestamp_ns"]))\
    /10**6
    datetime = obj ["datetime"]
    
    # if first record
    if (prev_seqnum == -1):
        reference_time = obj ["timestamp"]
    else:
        loss = obj ["payload"]["seqnum"] - prev_seqnum-1
    
    prev_seqnum    = obj ["payload"]["seqnum"]
    
    line_dict = {
        "delay_ms"          : delay_ms,
        "datetime"          : datetime,
        "loss"              : loss,
        "timestamp_s"       : (obj ["timestamp"]-reference_time)/10**9,
    }
    # pdb.set_trace()
    return line_dict
    

# parser of phy logs
# input: text row
# output list: dict [var] = [time] [val]

def parse_phy_log_line (line):
    global reference_time
    obj = json.loads(line)
    
    # if first record  
    if (reference_time == -1):
        reference_time = obj ["timestamp"]
    var_list = obj ["payload"] ["phy"].replace('\t','').split('\n')
    
    
    prefix = "phy_" 
    
    line_dict = {
        prefix+"timestamp_s"       : (obj ["timestamp"]-reference_time)/10**9,
    }
    
    # interface lost connection to AP
    if var_list [0] == "Not connected.":
        line_dict [prefix+"no_conn"] = True
        return line_dict
    # no data returned from iw
    if var_list [0] == "":
        line_dict [prefix+"no_data"] = True
        return line_dict
    
        
    for var in var_list:
        if (var!=''):
            title= var.split(':')[0]
            match title:
                case "SSID":
                    line_dict [prefix+"ssid"] = var.split(':')[1]
                case "signal":
                    line_dict [prefix+"rssi"] = int(var.split(':')[1]\
                    .replace(' ','')\
                    .replace('dBm',''))
                case "freq":
                    line_dict [prefix+"freq"] = int(var.split(':')[1]\
                    .replace(' ',''))
                case "RX":
                    
                    val   = var.split(':')[1]
                    
                    bytes = re.search("(\d)* bytes",val)
                    packets  = re.search("(\d)* packets",val)
                    
                    # you can now calculte delt packets and bytes at interface
                    # level as well as thyoughput
                    
                    line_dict [prefix+"rx_mb"] = int(bytes.group()\
                    .replace(" bytes", ""))*(10**-6)*8
                    line_dict [prefix+"rx_packets"] = int(packets.group()\
                    .replace(" packets", ""))
                    
                case "TX":
                    
                    val   = var.split(':')[1]
                    
                    bytes = re.search("(\d)* bytes",val)
                    packets  = re.search("(\d)* packets",val)
                    
                    line_dict [prefix+"tx_mb"] = int(bytes.group()\
                    .replace(" bytes", ""))*(10**-6)*8
                    line_dict [prefix+"tx_packets"] = int(packets.group()\
                    .replace(" packets", ""))
                
                case "rx bitrate":
                    # to do: extract HE-NSS and HE-GI and HE-DCM
                    val   = var.split(':')[1]
                    bitrate = re.search("(\d)*.(\d)* MBit/s",val).group()\
                    .replace(" MBit/s","")
                    mcs     = re.search("MCS (\d)*",val)
                    nss     = re.search("NSS (\d)*",val)
                    gi      = re.search("GI (\d)*",val)
                    
                    line_dict [prefix+"rx_bitrate"] = float (bitrate)
                    if (mcs):
                        line_dict [prefix+"rx_mcs"] = int (
                            mcs.group().replace("MCS ","")
                        )
                    else: 
                        line_dict [prefix+"rx_mcs"] = "no_data"
                    if (nss):
                        line_dict [prefix+"rx_nss"] = int (
                            nss.group().replace("NSS ","")
                        )
                    else: 
                        line_dict [prefix+"rx_nss"] = "no_data"
                    if (gi):
                        line_dict [prefix+"rx_gi"] = int (
                            gi.group().replace("GI ","")
                        )
                    else: 
                        line_dict [prefix+"rx_gi"] = "no_data"
                    
                case "tx bitrate":
                    # to do: extract HE-NSS and HE-GI and HE-DCM
                    val   = var.split(':')[1]
                    bitrate = re.search("(\d)*.(\d)* MBit/s",val).group()\
                    .replace(" MBit/s","")
                    mcs     = re.search("MCS (\d)*",val) 
                    nss     = re.search("NSS (\d)*",val) 
                    gi      = re.search("GI (\d)*",val) 
                    
                    line_dict [prefix+"tx_bitrate"] = float (bitrate)
                    if (mcs):
                        line_dict [prefix+"tx_mcs"] = int (
                            mcs.group().replace("MCS ","")
                        )
                    else: 
                        line_dict [prefix+"tx_mcs"] = "no_data"
                    if (nss):
                        line_dict [prefix+"tx_nss"] = int (
                            nss.group().replace("NSS ","")
                        )
                    else: 
                        line_dict [prefix+"tx_nss"] = "no_data"
                    if (gi):
                        line_dict [prefix+"tx_gi"] = int (
                            gi.group().replace("GI ","")
                        )
                    else: 
                        line_dict [prefix+"tx_gi"] = "no_data"
                case "bss flags":
                    val   = var.split(':')[1]
                    line_dict [prefix+"bss_flags"] = val.replace(" ","")
                case "dtim period":
                    val   = var.split(':')[1]
                    line_dict [prefix+"dtim_period"] = int(val.replace(" ",""))
                case "beacon int":
                    val   = var.split(':')[1]
                    line_dict [prefix+"beacon_int"] = int(val.replace(" ",""))
                    
    return line_dict
    
# parser of ap scan logs
# input: text row
# output list: dict [var] = [time] [val]

def parse_apscan_log_line (line):
    global reference_time
    obj = json.loads(line)
    
    # if first record  
    if (reference_time == -1):
        reference_time = obj ["timestamp"]
    var_list_string = obj ["payload"] ["ap_scan"].replace('\t','')
    
    prefix = "apscan_" 
    
    line_dict = {
        prefix+"timestamp_s"       : (obj ["timestamp"]-reference_time)/10**9,
    }
    
    # interface lost connection to AP
    if var_list_string == "Not connected.":
        line_dict [prefix+"no_conn"] = True
        return line_dict
    # no data returned from iw
    if var_list_string  == "":
        line_dict [prefix+"no_data"] = True
        return line_dict
    
    # split ap data into list
    finished = False
    res = re.search("BSS(.*)on wlp0s20f3\)",var_list_string)
    while (res!=None):
        var_list_string=var_list_string.replace(res.group(),'SPLIT_HERE')
        res = re.search("BSS(.*)on wlp0s20f3\)",var_list_string)
    var_list= var_list_string.split('SPLIT_HERE')
    
    
    ap_info = {
            prefix+"ssid"                   : [],
            prefix+"rssi"                   : [],
            prefix+"freq"                   : [],
            prefix+"station_count"          : [],
            prefix+"channel_utilisation"    : [],
        }
    line_dict.update({
        prefix+"2G":copy.deepcopy(ap_info),
        prefix+"5G":copy.deepcopy(ap_info),
        prefix+"6G":copy.deepcopy(ap_info),
    })
    for var in var_list:
        # on each ap, extract
        
        if (var!=''):
            ap_freq = 0

            info = var.split("\n")
            for i in info:
                if (i!=''):
                    title= i.split(':')[0]
                    
                    match title:
                        case "SSID":
                            ap_info [prefix+"ssid"]= \
                            i.split(':')[1].replace(' ','')
                        case " * station count":
                            ap_info [prefix+"station_count"]=\
                            int(i.split(':')[1].replace(' ',''))
                        case "signal":
                            ap_info [prefix+"rssi"]= \
                            i.split(':')[1]\
                            .replace(' ','')\
                            .replace('dBm','')
                        case "freq":
                            ap_freq = int(i.split(':')[1].replace(' ',''))
                            ap_info [prefix+"freq"]=ap_freq
                        case " * channel utilisation":
                            ch = i.split(':')[1].replace(' ','')
                            n = int(ch.split('/')[0])
                            d = int(ch.split('/')[1])
                            num = round(float(100*n/d),1)
                            ap_info [prefix+"channel_utilisation"]=num
                            
            ap_band = ""
            if  ap_freq <5000:
                ap_band = "2G"
            elif ap_freq <6000:
                ap_band = "5G"
            else:
                ap_band = "6G"
            
            for k in ap_info:
                val  = ap_info[k]
                if val == []:
                    line_dict[prefix+"no_data"]= "True"
                else:
                    line_dict[prefix+ap_band][k].append(val)
                
                
    return line_dict

# parser of ptp logs
# input: text row
# output list: dict [var] = [time] [val]

def parse_ptp_log_line (line):
    global reference_time
    obj = json.loads(line)
    
    # if first record  
    if (reference_time == -1):
        reference_time = obj ["timestamp"]
    var_list = obj ["payload"] ["ptp_status"].replace('\t','').split('\n')
    
    
    prefix = "ptp_" 
    
    line_dict = {
        prefix+"timestamp_s"       : (obj ["timestamp"]-reference_time)/10**9,
        prefix+"datetime"          : obj ["datetime"],
    }
    
    # interface lost connection to AP
    if var_list [0] == "Not connected.":
        line_dict [prefix+"no_conn"] = True
        return line_dict
    # no data returned from iw
    if var_list [0] == "":
        line_dict [prefix+"no_data"] = True
        return line_dict
    
        
    for var in var_list:
        if (var!=''):
            title= var.split(':')[0].replace(" ","")
            match title:

                case "DelayReqsent":
                    val   = int(var.split(':')[1].replace(" ",""))
                    line_dict [prefix+"delay_req_sent"] = val
                
                case "DelayRespreceived":
                    val   = int(var.split(':')[1].replace(" ",""))
                    line_dict [prefix+"delay_resp_received"] = val
                
                case "Syncreceived":
                    val   = int(var.split(':')[1].replace(" ",""))
                    line_dict [prefix+"sync_received"] = val
                
                case "Follow-upreceived":
                    val   = int(var.split(':')[1].replace(" ",""))
                    line_dict [prefix+"followups_received"] = val
                
                case "PTPEngineresets":
                    val   = int(var.split(':')[1].replace(" ",""))
                    line_dict [prefix+"resets"] = val
                
                case "State transitions":
                    val   = int(var.split(':')[1].replace(" ",""))
                    line_dict [prefix+"state_transitions"] = val
                
                case "OffsetfromMaster":
                    
                    readings   = var.split(':')[1].split(",")
                    current    = float(readings [0]\
                    .replace (" ","")\
                    .replace ("s",""))
                    line_dict [prefix+"offset_current_ms"] = current*1000
                    
                    if len(readings)==3:
                        mean    = float(readings [1]\
                        .replace (" ","")\
                        .replace ("mean","")\
                        .replace ("s",""))
                        dev    = float(readings [2]\
                        .replace (" ","")\
                        .replace ("dev","")\
                        .replace ("s",""))
                        line_dict [prefix+"offset_mean_ms" ]   = mean*1000                    
                        line_dict [prefix+"offset_dev_ms" ]    = dev*1000                    
                    
                    
                
                case "Clockcorrection":
                    
                    readings   = var.split(':')[1].split(",")
                    current    = float(readings [0]\
                    .replace (" ","")\
                    .replace ("ppm","")\
                    .replace ("(slewingatmaximumrate)",""))
                    line_dict [prefix+"clock_correction_current"] = current
                    if len(readings)==3:
                        mean    = float(readings [1]\
                        .replace (" ","")\
                        .replace ("mean","")\
                        .replace ("ppm",""))
                        dev    = float(readings [2]\
                        .replace (" ","")\
                        .replace ("dev","")\
                        .replace ("ppm",""))
                        line_dict [prefix+"clock_correction_mean" ]   = mean                    
                        line_dict [prefix+"clock_correction_dev" ]    = dev
                    
                    
                
                case "MeanPathDelay":
                    
                    readings   = var.split(':')[1].split(",")
                    current    = float(readings [0]\
                    .replace (" ","")\
                    .replace ("s",""))
                    line_dict [prefix+"path_delay_current_ms"] = current *1000
                    if len(readings)==3:
                        mean    = float(readings [1]\
                        .replace (" ","")\
                        .replace ("mean","")\
                        .replace ("s",""))
                        dev    = float(readings [2]\
                        .replace (" ","")\
                        .replace ("dev","")\
                        .replace ("s",""))
                        line_dict [prefix+"path_delay_mean_ms" ]   = mean * 1000                   
                        line_dict [prefix+"path_delay_dev_ms" ]    = dev * 1000
                    
                    
                    
    return line_dict
    

# parser of iperf logs
# input: text row
# output list: dict [var] = [time] [val]


def parse_iperf_log_line (line):
    
    prefix = "iperf_" 
    line_dict = {}
    
    for key in line.keys():
        if (key=="end"):
            line_dict[prefix+"timestamp_s"]       = line [key]
        if (key=="bits_per_second"):
            line_dict[prefix+"mbps"]       = int(line [key])*(10**-6)
        else:
            line_dict[prefix+key]       = line [key]
            
    return line_dict

#---------------------------- File parsers -------------------------------------

def parse_control_log(location, config, file):
        global data
        global prev_seqnum     
        global reference_time   
        # reset references 
        prev_seqnum     = -1
        reference_time  = -1
        prefix = "control_"
        
        data [location][config] = { 
            prefix+"datetime": [],
            prefix+"delay_ms": [],
            prefix+"loss": [],
            prefix+"timestamp_s": [],
            }
        
        f = open(file)
        lines = f.readlines()
        for line in lines:
            line_out ={}
            if line:
                line_out = parse_control_log_line(line)
                
                data [location][config][prefix+"datetime"] \
                .append(line_out["datetime"])
                data [location][config][prefix+"delay_ms"] \
                .append(line_out["delay_ms"])
                data [location][config][prefix+"loss"] \
                .append(line_out["loss"])                
                data [location][config][prefix+"timestamp_s"] \
                .append(line_out["timestamp_s"])
                
        f.close()
        

def parse_phy_log(location, config, file):
        global data
        global reference_time 
        # reset references 
        reference_time  =-1
        
        prefix = "phy_"
        
        base_obj = { 
                prefix+"timestamp_s":   [],
                prefix+"ssid":          [],
                prefix+"freq":          [],
                prefix+"rx_mb":         [],
                prefix+"rx_packets":    [],
                prefix+"tx_mb":         [],
                prefix+"tx_packets":    [],
                prefix+"rssi":          [],
                prefix+"rx_bitrate":    [],
                prefix+"rx_mcs":        [],
                prefix+"rx_nss":        [],
                prefix+"rx_gi":         [],
                prefix+"tx_bitrate":    [],
                prefix+"tx_mcs":        [],
                prefix+"tx_nss":        [],
                prefix+"tx_gi":         [],
                prefix+"bss_flags":     [],
                prefix+"dtim_period":   [],
                prefix+"beacon_int":    [],
            }
        # data [location][config].update (base_obj) 
        
        f = open(file)
        lines = f.readlines()
        for line in lines:
            line_out ={}
            if line:
                line_out = parse_phy_log_line(line)
                no_conn = False
                no_data = False
                if (prefix+'no_conn') in line_out.keys(): 
                    # no phy measurment captured
                    no_conn = True
                
                if (prefix+'no_data') in line_out.keys(): 
                    # no data returned from iw
                    no_data = True
                    
                for key in list(base_obj.keys()):
                    # add time stamp entry even if no data to indicate
                    # inteface was not connected
                    if key == prefix+'timestamp_s':
                        base_obj[key].append(line_out[key])
                        continue
                    try:
                        if no_conn:
                            base_obj[key].append("no_conn")
                        elif no_data: 
                            base_obj[key].append("no_data")
                        else:
                            base_obj[key].append(line_out[key])
                    except:
                        pdb.set_trace()
        data [location][config].update(base_obj)
        f.close()

def parse_apscan_log(location, config, file):
        global data
        global reference_time 
        # reset references 
        reference_time  =-1
        
        prefix = "apscan_"
        
        base_obj_1 = { 
                prefix+"timestamp_s":   [],
                prefix+"ssid":          [], # these vars will be 2D arrays
                prefix+"freq":          [],
                prefix+"rssi":          [],
                prefix+"station_count": [],
                prefix+"channel_utilisation":        [],
                
            }
        
        base_obj = { 
                prefix+"2G":           [],
                prefix+"5G":           [], # these vars will be 2D arrays
                prefix+"6G":           [],
            }
        # data [location][config].update (base_obj) 
        
        f = open(file)
        lines = f.readlines()
        for line in lines:
            line_out ={}
            if line:
                line_out = parse_apscan_log_line(line)
                no_conn = False
                no_data = False
                if (prefix+'no_conn') in line_out.keys(): 
                    # no phy measurment captured
                    no_conn = True
                
                if (prefix+'no_data') in line_out.keys(): 
                    # no data returned from iw
                    no_data = True
                    
                for key in list(base_obj.keys()):
                    # add time stamp entry even if no data to indicate
                    # inteface was not connected
                    try:
                        if key == prefix+'timestamp_s':
                            base_obj[key].append(line_out[key])
                            continue
                        if no_conn:
                            base_obj[key].append("no_conn")
                        elif no_data:
                            continue
                            # base_obj[key].append("no_data")
                        else:
                            base_obj[key].append(line_out[key])
                    except:
                        pdb.set_trace()
        data [location][config].update(base_obj)
        f.close()

def parse_ptp_log(location, config, file):
        global data
        global reference_time 
        # reset references 
        reference_time  =-1
        
        prefix = "ptp_"
        
        base_obj = { 
                prefix+"timestamp_s":                       [],
                prefix+"datetime":                          [],
                prefix+"clock_correction_current":          [],
                prefix+"clock_correction_mean":             [],
                prefix+"clock_correction_dev":              [],
                prefix+"offset_current_ms":                    [],
                prefix+"offset_mean_ms":                       [],
                prefix+"offset_dev_ms":                        [],
                prefix+"path_delay_current_ms":             [],
                prefix+"path_delay_mean_ms":                [],
                prefix+"path_delay_dev_ms":                 [],
                prefix+"delay_req_sent":                    [],
                prefix+"delay_resp_received":               [],
                prefix+"sync_received":                     [],
                prefix+"resets":                            [],
                prefix+"state_transitions":                 [],
                prefix+"followups_received":                 [],
            }
        # data [location][config].update (base_obj) 
        
        f = open(file)
        lines = f.readlines()
        for line in lines:
            line_out ={}
            if line:
                line_out = parse_ptp_log_line(line)
                no_conn = False
                no_data = False
                if (prefix+'no_conn') in line_out.keys(): 
                    # no phy measurment captured
                    no_conn = True
                
                if (prefix+'no_data') in line_out.keys(): 
                    # no data returned from iw
                    no_data = True
                    
                for key in list(base_obj.keys()):
                    # add time stamp entry even if no data to indicate
                    # inteface was not connected
                    if key == prefix+'timestamp_s':
                        base_obj[key].append(line_out[key])
                        continue
                    try:
                        if no_conn:
                            base_obj[key].append("no_conn")
                        elif no_data or (key not in line_out.keys()): 
                            base_obj[key].append("no_data")
                        else:
                            base_obj[key].append(line_out[key])
                    except:
                        pdb.set_trace()
        data [location][config].update(base_obj)
        f.close()

def parse_iperf_log(location, config, file):
        global data
        
        
        prefix = "iperf_"
        
        base_obj = { 
                prefix+"timestamp_s":        [],
                prefix+"socket":			 [],
                prefix+"start":			     [],
                prefix+"seconds":			 [],
                prefix+"bytes":			     [],
                prefix+"mbps":	             [],
                prefix+"retransmits":		 [],
                prefix+"snd_cwnd":			 [],
                prefix+"rtt":			 	 [], # in us according to iperf source code
                prefix+"rttvar":			 [], # in us according to iperf source code
                prefix+"pmtu":			 	 [],
                prefix+"omitted":			 [],
                prefix+"sender": 			 []
            }
        
        f = open(file)
        json_content = f.readline()
        
        if (json_content==''):
            pdb.set_trace()
            data [location][config] = "no_data"
            return None
        obj = json.loads(json_content)

        # load json
        
        if "intervals" not in obj.keys():
            # if there are no intervals, return None
            pdb.set_trace()
            data [location][config] = "no_data"
            return None
        
        # if there are intervals, go on each interval, pass json log to  
        # parse_iperf_log_line
        i= 0 
        while i < len(obj["intervals"]):
            line_out = parse_iperf_log_line(obj["intervals"][i]["streams"][0])
            i        = i+1
            no_conn  = False
            no_data  = False
            
            if (prefix+'rtt') in line_out.keys(): 
                line_out [prefix+'rtt'] = line_out [prefix+'rtt'] /1000 # convert to ms 
            if (prefix+'rttvar') in line_out.keys(): 
                line_out [prefix+'rttvar'] = line_out [prefix+'rtt'] /1000 # convert to ms
            
            if (prefix+'no_conn') in line_out.keys(): 
                # no phy measurment captured
                no_conn = True
            
            if (prefix+'no_data') in line_out.keys(): 
                # no data returned from iw
                no_data = True
                
            for key in list(base_obj.keys()):
                # add time stamp entry even if no data to indicate
                try:
                    if key == prefix+'timestamp_s':
                        base_obj[key].append(line_out[key])
                        continue
                    if no_conn:
                        base_obj[key].append("no_conn")
                    elif no_data or (key not in line_out.keys()): 
                        base_obj[key].append("no_data")
                    else:
                        base_obj[key].append(line_out[key])
                except:
                    pdb.set_trace()
        data [location][config].update(base_obj)
        f.close()

def parse_logs ():

    # 4 location x 9 configs x 5 files per = 

    # loop on locations, configs, log files

    location = 1
    total_files = 0
    while location<=locations:
        j=0
        while j<len(configs):
            
            # for each file, loop on rows
            
            # control
            file_name = '../logs/log_'+expid+'_'+configs[j]+'_'+runs[j]+'_loc'+\
            str(location) +'.json'
            total_files += 1 
            print (total_files, " processing: " , file_name)
            
            parse_control_log (location, configs[j], file_name)

            # phy 
            
            file_name = '../logs/log_'+expid+'_'+configs[j]+'_tcp_single_UL_'\
            +runs[j]+'_loc'+ str(location) +'_phy.json'
            
            total_files += 1 
            print (total_files, " processing: " , file_name)
            
            parse_phy_log (location, configs[j], file_name)            
            
            # ap scan 
            
            file_name = '../logs/log_'+expid+'_'+configs[j]+'_tcp_single_UL_'\
            +runs[j]+'_loc'+ str(location) +'_ap_scan.json'
            
            total_files += 1 
            print (total_files, " processing: " , file_name)
            
            parse_apscan_log (location, configs[j], file_name)  
            
            # ptp
            
            file_name = '../logs/log_'+expid+'_'+configs[j]+'_tcp_single_UL_'\
            +runs[j]+'_loc'+ str(location) +'_ptp_status.json'
            
            total_files += 1 
            print (total_files, " processing: " , file_name)
            
            parse_ptp_log (location, configs[j], file_name)              
            
            
            # iperf
            # perama_site_n_20mhz_24ghz_run1_tcp_single_UL_loc4.json
            
            file_name = '../logs/'+expid+'_'+configs[j]+'_'+runs[j]+\
            '_tcp_single_UL_loc'+ str(location) +'.json'
            
            total_files += 1 
            print (total_files, " processing: " , file_name)
            
            parse_iperf_log (location, configs[j], file_name)
            
            
            
            # append var reading to the time series in global containers

            # dat container [location] [config] [var]. append (dict [var])

            j=j+1
                    
        location=location+1
 

def store_output (file_name):
    location = 1
    while location<=locations:
        j=0
        while j<len(configs):
            try:
                s = json.dumps(data[location][configs[j]])
            except:
                print(traceback.format_exc())
                pdb.set_trace()
            j=j+1
        location=location+1
    data_json = json.dumps(data)
    with open(file_name, 'w') as outfile:
        json.dump(data, outfile)
    
    print ("output stored in: ", file_name)

#---------------------------- main program -------------------------------------

initialize ()

parse_logs ()

store_output ("perama_range_testing.json")

print ("Done.")

sys.exit()