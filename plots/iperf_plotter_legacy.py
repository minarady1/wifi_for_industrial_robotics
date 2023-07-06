# -*- coding: utf-8 -*-
'''
Plotting script for captured measurements

PLotter v2.0

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



#--------------------------- Global variables ---------------------------------

duration = 300 #s 
expid = "cortex"
runid = "run8"
phy_data_lookup = False
current_location = ""

if len (sys.argv)>1:
    expid = sys.argv[1]
    runid = sys.argv[2]

# labels of phy_streamtype statistics

labels = [] 

# labels of global phy statistics

labels_phy = []

locations= [
    "cortex", # TPLink
    "cortex_asus", # Asus
    # "office",
    ]
locations_labels= [
    "AP#2",
    "AP#1",
    # "Office",
    ]

exp_labels_streaming_pdr = [

              "UDP UL High (Rx)",            
              "UDP UL Low (Rx)",
              "UDP DL High",
              "UDP DL Low",
              ]

exp_labels_control = ["TCP  UL",  
              "TCP UL/DL",  
              ]

stream_configs = [
    # "tcp_DL_cwin100_server",
    # "tcp_DL_cwin200_server",
    # "tcp_DL_cwin400_server",
    
    # "tcp_DL_bw600_server",
    # "tcp_DL_bw1200_server",
    # "tcp_DL_bw1800_server",
    
    "tcp_single_UL",
    # "tcp_single_UL_server",
    
    "tcp_single_DL",
    # "tcp_single_DL_server",
    
    "tcp_dual_UL",
    # "tcp_dual_UL_server",
    
    "tcp_dual_DL",
    # "tcp_dual_DL_server",
    
    # "udp_single_UL",
    # "udp_single_UL_server",
    
    # "udp_single_DL",
    # "udp_single_DL_server",
    
    # "udp_dual_UL",
    # "udp_dual_UL_server",
    
    # "udp_dual_DL",
    # "udp_dual_DL_server",
 ]

stream_labels = [
    # "Cwin 100 kB",
    # "Cwin 200 kB",
    # "Cwin 400 kB",
    
    # "Target BW 600Mbps",
    # "Target BW 1200Mbps",
    # "Target BW 1800Mbps",
    
    
    "Uplink",
    # "tcp_single_UL_server",
    
    # "Downlink",
    "Downlink",
    
    "Shared Uplink",
    # "tcp_dual_UL_server",
    
    # "Shared Downlink",
    "Shared Downlink",
    
    # "UDP UL",
    # "UDP UL",
    
    # "UDP DL",
    # "UDP DL",
    
    # "udp_dual_UL",
    # "udp_dual_UL_server",
    
    # "udp_dual_DL",
    # "udp_dual_DL_server",
              ]

phy_configs = [
    # "ax_80mhz_5ghz_cwin_config",
    "ax_80mhz_5ghz",
    "ac_80mhz_5ghz",
    "ax_20mhz_5ghz",
    "ax_20mhz_24ghz",
    "ac_20mhz_5ghz",
    "n_20mhz_5ghz",
    "n_20mhz_24ghz",
    
    # "a_20mhz_5ghz", 
    # "g_20mhz_24ghz",
    ]


# % AX 80 Mhz 5 GHz {'IEEE80211_MODE_11AXA_HE80'} 1201
# % AC 80 Mhz 5 hz {'IEEE80211_MODE_11AC_VHT80'} 866
# % AX 20 Mhz 5 GHz {'IEEE80211_MODE_11AXA_HE20'} 286
# % AX 20 Mhz 2.4 GHz {'IEEE80211_MODE_11AXG_HE20'} 286
# % AC 20 Mhz 5 GHz {'IEEE80211_MODE_11AC_VHT20'} 192
# % N 20 Mhz 5 GHz {'IEEE80211_MODE_11NA_HT20'} 144
# % N 20 Mhz 2.4GHz {'IEEE80211_MODE_11NG_HT20'} 144

phy_configs_labels = [
    "AX 80 Mhz 5 GHz\n1201 Mbps",
    "AC 80 Mhz 5 GHz\n866 Mbps",
    "AX 20 Mhz 5 GHz\n286 Mbps",
    "AX 20 Mhz 2.4 GHz\n286 Mbps",
    "AC 20 Mhz 5 GHz\n192 Mbps",
    "N 20 Mhz 5 GHz\n144 Mbps",
    "N 20 Mhz 2.4GHz\n144 Mbps",
    # "a_20mhz_5ghz", 
    # "g_20mhz_24ghz",
    ]

line_styles= [
     'solid',
     'dashed',
     'solid',
     'solid',
     'dashed',
     'solid',
     'dashed',
    ]
marker_styles= [
     '.',
     '.',
     '+',
     'x',
     '_',
     'x',
     '.',
    ]

colors= [
     'maroon',
     'indianred',
     'royalblue',
     'navy',
     'tab:orange',
     'forestgreen',
     'dimgrey',
    ]


#--------------------------- Declarations -------------------------------------

throughput_all = []
throughput_global = {}
throughput_90th_all = {}
pdr_90th_all = {}
txrate_global = {}

streaming_time_all  = []
phy_time_all  = []
pdr_all = []
pdr_labels = []
tcp_labels = []
retransmits_all = []
cwnd_all = []
rtt_all = []

json_objects_data = []
data=[]
phy_df = pd.DataFrame()
time  = []
txrate_all = []
rxrate_all = [] 
mode_all  = []
rssi_all = []




def collectStreamData (grouping, filter_params):
    global runid
    global stream_configs
    global phy_configs
    global json_objects_data
    global labels
    global labels_phy
    global phy_df
    current_location = filter_params["location"]
    files = []
    global files_global 
    
    
    if (grouping == "phy_stream_config"):
        i = 0
        for config in stream_configs:
            server_dir = ""
            if "server" in config:
                server_dir = "server/"
            j = 0
            for phy in phy_configs:
                labels.append("{} {}".format(stream_labels[i], phy_configs[j]))
                j = j +1
                files.append("../logs/{}/{}{}_{}_{}_{}.json".format(
                    runid,server_dir,filter_params["location"], phy ,runid, 
                    config))
            i = i +1
    if (grouping == "stream_config"):
        i = 0
        for config in stream_configs:
            if config != filter_params["stream_config"]:
                i = i +1
                continue
            server_dir = ""
            if "server" in config:
                server_dir = "server/"
            j = 0
            for phy in phy_configs:
                labels.append("{}".format(phy_configs_labels[j]))
                labels_phy.append(phy)
                files.append("../logs/{}/{}{}_{}_{}_{}.json".format(
                        runid, server_dir, filter_params["location"], phy, 
                        runid, config))
                j=j+1
            i = i +1

    for file in  files:
        f = open(file)
        line = f.readline()
        if line:
            json_objects_data.append(json.loads(line))
        f.close()
    
    return json_objects_data

def collectPhyData ():
    global runid
    global phy_df
    
    phy_dir = "../logs/{}/phy/phy_log.txt".format(runid)
    phy_df = pd.DataFrame(columns = ['timestamp', 'txrate', 'rxrate',
                             'rssi', 'mode'])

    # print (file)
    f = open(phy_dir)
    lines= f.readlines()
    i = 0
    for line in lines:        
        if line:
            js = json.loads(line)
            phy_df = phy_df.append({'timestamp': js["timestamp"],
                       'txrate' : js["txrate"],
                       'rxrate' : js["rxrate"],
                       'rssi': js["rssi"],
                       'mode': js["mode"]}, ignore_index = True)
            i = i+1
    f.close()


def reset():
    global streaming_time_all  
    global throughput_all 
    global phy_time_all 
    global pdr_all 
    global pdr_labels 
    global tcp_labels 
    global data
    global labels
    global txrate_all
    global rxrate_all
    global mode_all
    global rssi_all
    global retransmits_all
    global cwnd_all
    global rtt_all
    global locations
    global json_objects_data
    
    throughput_all = []
    streaming_time_all  = []
    phy_time_all  = []
    pdr_all = []
    txrate_all = []
    rxrate_all = [] 
    mode_all  = []
    rssi_all = []
    pdr_labels= []
    tcp_labels= []
    data=[]
    labels = []
    retransmits_all = []
    cwnd_all = []
    rtt_all = []
    json_objects_data = []
    
def processData (filter_params):
    global json_objects_data
    global throughput_all 
    global throughput_90th_all
    global throughput_global
    global pdr_90th_all
    global txrate_global
    global streaming_time_all  
    global phy_time_all
    global pdr_all 
    global duration
    global phy_df
    global txrate_all
    global rxrate_all
    global rssi_all
    global mode_all
    global retransmits_all
    global rtt_all
    global cwnd_all
    current_location = filter_params ["location"]
    c = 0
    for exp in json_objects_data:
        time  = []
        throughput = []
        retransmits = []
        cwnd = []
        rtt = []
        pdr = []
        isUDP = False
        isTCP  = False
        ts_start = exp['start']['timestamp']['timesecs']
        ts_end = ts_start+duration

        for i in exp['intervals']:
            time.append (i ['sum']['start'])
            throughput.append (i ['sum']['bits_per_second']/ 10**6)
            if "retransmits" in i ['sum']:
                isTCP = True
                retransmits.append(i ['sum']['retransmits'])
                cwnd.append(i ['streams'][0]['snd_cwnd']/10**3)
                rtt.append (i['streams'][0]['rtt']/10**3)
            
            if "lost_percent" in i ['sum']:
                isUDP = True
                pdr.append(100 - i ['sum']['lost_percent'])
        
        throughput_all.append(throughput)            
        streaming_time_all.append(time)
        
        throughput_global [current_location] [labels_phy[c]] [filter_params ["stream_config"]]= throughput
        
        # add the 90th percentile to the global values of this PHY
        
        if (labels_phy[c] in throughput_90th_all[current_location].keys()):
            throughput_90th_all [current_location] [labels_phy[c]].append(np.percentile(throughput,90))
        else:
            throughput_90th_all [current_location] [labels_phy[c]] = [np.percentile(throughput,90)]
        
        if (isUDP):
            pdr_labels.append(labels[c])
            pdr_all.append(pdr)
            
            if (labels_phy[c] in pdr_90th_all.keys()):
                pdr_90th_all [labels_phy[c]].append(np.percentile(pdr,90))
            else:
                pdr_90th_all[labels_phy[c]] = [np.percentile(pdr,90)]
        
        if (isTCP):
            tcp_labels.append(labels[c])
            retransmits_all.append(retransmits)
            cwnd_all.append(cwnd)
            rtt_all.append(rtt)
        
        if (phy_data_lookup):
            
            res = phy_df.query("{}< timestamp <{}".format(ts_start, ts_end))
            phy_time = res ["timestamp"].tolist()
            try:
                st = phy_time[0]
            except:
                pdb.set_trace()
            interval = phy_time[1] - phy_time[0]  
            # shift the timestamps to end of the interval window
            phy_time = [(x - st + interval) for x in phy_time]
            txrate = res ["txrate"].tolist()
            rxrate = res ["rxrate"].tolist()
            rssi   = res ["rssi"].tolist()
            mode   = res ["mode"].tolist()
            print (">>", labels[c], set(mode), max(txrate) )
            phy_time_all.append (phy_time)
            txrate_all.append(txrate)
            rxrate_all.append(rxrate)
            rssi_all.append(rssi)
            mode_all.append(mode)
            
            # add the txrate of this phy
            if (labels_phy[c] in txrate_global.keys()):
                txrate_global [labels_phy[c]].extend(txrate)
            else:
                txrate_global[labels_phy[c]] = txrate
        c= c+1
            
def plotViolin (data, labels, xlimit, ylabel, tag , ylimit=None, step = None):
    # justifying the data
    i =0
    while (i< len(data)):
        data [i]= data[i][0:xlimit]
        i= i+1
    
    # plot:
    fig, ax = plt.subplots()
    ticks =  list(range(1,len(labels)+1))
    try:
        vp = ax.violinplot(data, ticks)
    except:
        pdb.set_trace()
    # styling:
    for body in vp['bodies']:
        body.set_edgecolor('#000000')
        body.set_alpha(0.4)
    
    plt.xlabel('Configuration')
    ax.set_xticks(ticks)
    if (ylimit!=None):
        ax.set_yticks(np.arange(0,ylimit,step))

    plt.ylabel(ylabel)
    ax.grid(True)
    ax.set_xticklabels(labels, rotation=90, size = 10)
    plt.title ("{} {}".format(tag,runid))
    plt.savefig(tag+"_violin.png", dpi=400, bbox_inches='tight', rotation = 90)
    

def plotBox (data, labels, xlimit, ylabel, tag , ylimit=None, step = None):

    # merging the columns for each location.
    i =0
    
    # justifying the data to the set max limit
    while (i< len(data)):
        data [i]= data[i][0:xlimit]
        i= i+1
   
    try:
        df = pd.DataFrame(data, index=labels)
    except:
        pdb.set_trace()
    fig, ax = plt.subplots()
    try:
        df.T.boxplot(vert=False)
        plt.subplots_adjust(left=0.25)
    except:
        pdb.set_trace()

    plt.ylabel('Configuration')
    plt.xlabel(ylabel)
    #ax.set_xticks(ticks)
    if (ylimit!=None):
        ax.set_yticks(np.arange(0,ylimit,step))

    ax.grid(True)
    #ax.set_xticklabels(labels, rotation=90, size = 10)
    plt.title ("{} {}".format(tag,runid))
    plt.savefig(tag+"_box.png", dpi=400, bbox_inches='tight', rotation = 90)


def plotGlobal (data_dict, ylabel, tag, stream ):
    global phy_configs
    global_values = []
    global_keys   = []
    
    # plot:
    i = 0
    for c in phy_configs:
        j = 0
        for l in locations:
            if (j%2==0): 
                global_keys.append ("")
            else:
                global_keys.append (phy_configs_labels [i])
                
            global_values.append ( list(data_dict[l][c][stream]))
            j=j+1
        i= i+1

    df = pd.DataFrame(global_values, index=global_keys)
    # fig, ax = plt.subplots(figsize=(12, 8))
    
    fig, ax = plt.subplots()
    bp_dict = df.T.boxplot(vert=False, return_type='both', patch_artist = True)
    plt.subplots_adjust(left=0.25)

    # adding manual legend
    legend_elements = [
        Line2D([0], [0], color='b', label= "Asus AX-GTE11000"),
        Line2D([0], [0], color='g', label='D-Link-X8630')
        ]
    
    ax.legend(handles=legend_elements, loc='best')
    i = 0
    
    for (box, median) in zip ( bp_dict[1]["boxes"], bp_dict[1]["medians"]):
        box.set_facecolor("w")
        median.set_color ("black")
        if (i%2 ==0):
            box.set_edgecolor("g")
        else:
            box.set_edgecolor("b")
        i = i+1
    
    i = 0
    while (i < len (bp_dict[1]["whiskers"])):
        if (i%4 ==0):
            bp_dict[1]["whiskers"][i].set_color("g")
            bp_dict[1]["whiskers"][i+1].set_color("g")
        else:
            bp_dict[1]["whiskers"][i].set_color("b")
            bp_dict[1]["whiskers"][i+1].set_color("b")
        i = i+2
    plt.ylabel('Configuration')
    plt.xlabel(ylabel)
    plt.xlim([0,200])
    # plt.xticks( np.arange(0,280,10))
    plt.grid(True)
    plt.savefig(tag+"_box_global.png",bbox_inches='tight', dpi=400, rotation = 90)
    

def plotGlobalPercentiles (data_dict, ylabel, tag ):
    global phy_configs
    global_values = []
    global_keys   = []
    
    # plot:
    i=0
    for c in phy_configs:
        j = 0
        for l in locations:
            if (j%2==0): 
                global_keys.append ("")
            else:
                global_keys.append (phy_configs_labels [i])
            
            global_values.append ( list(data_dict[l][c]))
            
            j=j+1
        i = i+1

    df = pd.DataFrame(global_values, index=global_keys)
    fig, ax = plt.subplots()
    bp_dict = df.T.boxplot(vert=False, return_type='both', patch_artist = True)
    plt.subplots_adjust(left=0.25)
        # adding manual legend
    
    legend_elements = [
        Line2D([0], [0], color='b', label= "Asus AX-GTE11000"),
        Line2D([0], [0], color='g', label='D-Link-X8630')
        ]
    
    ax.legend(handles=legend_elements, loc='best')
    
    i=0
    for (box, median) in zip ( bp_dict[1]["boxes"], bp_dict[1]["medians"]):
        box.set_facecolor("w")
        median.set_color ("black")
        if (i%2 ==0):
            box.set_edgecolor("g")
        else:
            box.set_edgecolor("b")
        i = i+1
    
    i = 0
    while (i < len (bp_dict[1]["whiskers"])):
        if (i%4 ==0):
            bp_dict[1]["whiskers"][i].set_color("g")
            bp_dict[1]["whiskers"][i+1].set_color("g")
        else:
            bp_dict[1]["whiskers"][i].set_color("b")
            bp_dict[1]["whiskers"][i+1].set_color("b")
        i = i+2
    plt.ylabel('Configuration')
    plt.xlabel(ylabel)
    plt.xlim([0,200])
    #ax.set_xticks(ticks)

    ax.grid(True)
    #ax.set_xticklabels(labels, rotation=90, size = 10)
    # plt.title ("{} {}".format(tag,runid))
    plt.savefig(tag+"_box_global_per.png", dpi=400, bbox_inches='tight', rotation = 90)
    
def plotTimeseries (data, time, labels, xlimit, ylabel, tag , ylimit=None, stepx = None, stepy = None):
    
    # plot:
    plt.figure(figsize=(12, 8))
    
    i = 0
    while (i < len(data)):
        plt.plot(time [i][0:xlimit], data [i][0:xlimit], linewidth=1.5, label = labels [i],
        linestyle= line_styles[i], marker = marker_styles[i], color = colors [i])
        i =i+1
    
    plt.tight_layout()
    if (ylimit!= None and stepy !=None):
        plt.yticks( np.arange(ylimit[0],ylimit[1],stepy))
        plt.ylim (ylimit)
    plt.ylabel(ylabel)
    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid(True)
    # plt.title ("{} {}".format(tag,runid))
    plt.savefig(tag+"_time.png", dpi=300, bbox_inches='tight')
    
    #plt.show()

def plotCDF (data, labels, timelimit, tag ,xlabel, ylabel,  xlimit = None, ylimit=None, stepy = None, stepx = None,
    legend_loc = None):
    
    plt.figure()
    
    i = 0
    while (i < len(data)):
        count, bins_count = np.histogram(data [i][0:timelimit], bins=30)
        pdf = count / sum(count)
        cdf = np.cumsum(pdf)
        plt.plot(bins_count[1:],cdf,label= labels [i],
                 linestyle= line_styles[i], marker = marker_styles[i],color = colors [i] )
        i =i+1
    if (ylimit):
        plt.ylim(ylimit)
    if (stepy):
        plt.yticks( np.arange(ylimit[0],ylimit[1],stepy))
    if (xlimit):
        plt.xlim(xlimit)
    if (stepx):
        plt.xticks( np.arange(xlimit[0],xlimit[1],stepx))
    plt.tight_layout()
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if (legend_loc):
        plt.legend(loc=legend_loc)
    else:
        plt.legend( )
    plt.grid(True)
    #plt.title ("{} {}".format(tag,runid))
    plt.savefig(tag+"_cdf.pdf", dpi=300, bbox_inches='tight')



def plotPDF (data, labels, xlimit, ylabel, tag , ylimit=None, step = None):
    
    plt.figure(figsize=(6, 6))
    fig, ax = plt.subplots()
    i = 0
    while (i < len(data)):
        count, bins_count = np.histogram(data [i][0:xlimit], bins=10)
        pdf = count / sum(count)
        ax.bar(bins_count[1:]+((1/3)*i),pdf,label= labels [i], linewidth = 2, alpha = 0.5, width = 1/3)
        i =i+1
    
    
    plt.xlabel(ylabel)
    plt.ylabel("PDF")
    plt.legend(phy_configs)
    plt.grid(True)
    plt.title ("{} {}".format(tag,runid))
    plt.tight_layout()
    plt.savefig(tag+"_pdf.png", dpi=300, bbox_inches='tight')

def plotPDFMultiple (data, labels, datalimit, ylabel, tag , xlimit = None, ylimit=None, step = None):
    
    
    figure, axis = plt.subplots(len(data), 1, sharex = True)
    i = 0
    #pdb.set_trace()
    while (i < len(data)):
        count, bins_count = np.histogram(data [i][datalimit[0]:datalimit[1]], bins=10)
        pdf = count / sum(count)
        axis[i].bar(bins_count[1:],pdf,label= labels [i], )
       # axis[i].legend()
        axis[i].grid(True)
        axis[i].set_xlim(xlimit)
        if (ylimit != None):
            axis[i].set_ylim(ylimit)
        axis[i].set_title(labels [i])
        i =i+1
    #pdb.set_trace()
    if (xlimit[1]<xlimit[0]):
        plt.setp(axis, xticks=list(np.arange(xlimit[1],xlimit[0],step)))
    else:
        plt.setp(axis, xticks=list(np.arange(xlimit[0],xlimit[1],step)))
    plt.xlabel(ylabel)
    figure.text(0.04, 0.5, 'PDF', va='center', rotation='vertical')
    # plt.title ("{} {}".format(tag,runid))
    # plt.tight_layout()
    plt.show()
    plt.savefig(tag+"_pdfmulti.png", dpi=300)
  
    
def plot(groupby, filter_params):
    global duration
    reset()
    
    collectStreamData(groupby, filter_params)
    processData(filter_params)
    
  
    # plotBox(throughput_all, labels, duration,  
    #             'Throughput (Mbps)', 
    #             filter_params["tag"]+"_throughput")

    # plotCDF(throughput_all, labels, duration,  
                # 'Throughput (Mbps)', 
                # filter_params["tag"]+"_throughput")    

    # plotTimeseries(throughput_all, streaming_time_all, labels, duration,  
                # 'Throughput (Mbps)', 
                # filter_params["tag"]+"_throughput",
                # stepy = 20 )
                
    
    if (phy_data_lookup):
        # plotPDFMultiple(rssi_all, labels, [0,300], 
              # 'RSSI (dBm)', 
              # filter_params["tag"]+"_rssi", [-50,-60], [0,1], 1 )
              
        # plotCDF(txrate_all, labels, duration, 
                      # filter_params["tag"]+"_txrate",
                      # xlabel = "PHY bit-rate (Mbps)",
                      # ylabel = "Ratio of Traffic",
                      # stepx  = 100,
                      # stepy  = 0.1,
                      # xlimit = [0, 1210],
                      # ylimit = [0,1.1],
                      # )
        # plotTimeseries(txrate_all, phy_time_all, labels, duration, 
                      # "PHY bit-rate (Mbps)",
                      # filter_params["tag"]+"_txrate",
                      # ) 
    
        plotCDF(rxrate_all, labels, duration, 
                  filter_params["tag"]+"_rxrate",
                  xlabel = "PHY bit-rate (Mbps)", 
                  ylabel = "Ratio of Traffic",
                  stepx  = 100,
                  stepy  = 0.1,
                  xlimit = [0, 1210],
                  ylimit = [0,1.1])

    
    

           
        
    # if (len(pdr_all)>0):
        # plotBox(pdr_all, pdr_labels, duration,  'PDR (%)',  filter_params["tag"]+ "_pdr")

    # if (len(retransmits_all)>0):
        # plotCDF(retransmits_all, tcp_labels, duration,  
                # filter_params["tag"]+ "_retries", 
                # ylimit = [0,1.1], 
                # xlimit = [0,17],  
                # stepy = 0.1, 
                # stepx = 1,
                # legend_loc = "lower right",
                # xlabel = "Retries",
                # ylabel = "Ratio of Traffic")
        # plotTimeseries(retransmits_all, streaming_time_all, labels, duration, 
                       # "Retries",
                      # filter_params["tag"]+"_retries",
                      # ) 
    # if (len(rtt_all)>0):
        # plotCDF(rtt_all, tcp_labels, duration,   
                # filter_params["tag"]+ "_rtt",
                # xlabel = "RTT (ms)",
                # ylabel = "Ratio of Traffic")

    # if (len(cwnd_all)>0):
        # plotCDF(cwnd_all, tcp_labels, duration,  'Congestion Window (kB)',  
                # filter_params["tag"]+ "_cwnd")
        # plotTimeseries(cwnd_all, streaming_time_all, labels, duration,  
                # 'Congestion Window (kB)', 
                # filter_params["tag"]+"_cwindow", ylimit= [0,160], stepx = 1, stepy = 10)
    

# initializing global data holders

for loc in locations: 
    throughput_90th_all [loc] = {}
    throughput_global [loc]   = {}
    for phy in phy_configs: 
        throughput_global [loc] [phy] = {}
        for s in stream_configs:
            throughput_global [loc] [phy] [s] = []


if (phy_data_lookup):
    collectPhyData()

tag_template = "{}_stream_{}_{}"
i = 0
while (i<len(locations)):
    print (locations[i])
    j = 0   
    while (j<len(stream_configs)):
        
        plot("stream_config", {
            "location":locations[i],
            "stream_config": stream_configs[j],
            "tag": tag_template.format(stream_configs[j],
            runid,
            locations[i])})
        
        
        j = j+1
    i=i+1
    
# all


# j = 0   
# while (j<len(stream_configs)):
#     plotGlobal(throughput_global,"Mbps", "{}_stream_{}".format(stream_configs[j],
#         runid,)+"_throughput" , 
#         stream_configs[j])
#     j=j+1


plotGlobalPercentiles(throughput_90th_all,"Mbps", "throughput_90th")
# same for retries
# same for tx/rx rate
    
# plotGlobal(pdr_90th_all,"%", "pdr_90th")

# plotGlobal(txrate_global,"Mbps", "txrate_global")
print ("Done.")
sys.exit()