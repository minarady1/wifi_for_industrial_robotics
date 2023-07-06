# -*- coding: utf-8 -*-
'''
Plotter v3.0

Plotting script for captured measurements

Input: json file containing time series data per variable per config per 
    location
Output: 
    - Time seris plot of each variable aggregate by location (all configs)
    - Box plots of throughput, delay aggregated by location

    
    

Bugwright2 Project
Mina Rady mina.rady@insa-lyon.fr

'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from statistics import mean

import json
import sys
import pdb
import pandas as pd
import time

pd.set_option('display.max_rows', None)
#==========================- Declarations ======================================

# expid, runid, duration, number of locations

# list of 80211 configs

# ax (Wi-Fi 6E) 6 GHz 160 MHz, 1254 Mbps
# ax (Wi-Fi 6) 5 GHz 80 MHz, 961 Mbps
# ac (Wi-Fi 5) 5 GHz 80 MHz, 961 Mbps
# ax (Wi-Fi 6E) 6 GHz 80 MHz, 865 Mbps
# ax (Wi-Fi 6) 2.4 GHz 20 MHz, 286 Mbps
# ax (Wi-Fi 6) 5 GHz 20 MHz, 258 Mbps
# ac (Wi-Fi 5) 5 GHz 20 MHz, 244 Mbps
# ax (Wi-Fi 6E) 6 GHz 20 MHz, 229 Mbps
# n (Wi-Fi 4) 2.4 GHz 20 MHz, 144 Mbps

configs_labels = [
"ax/6/160 ", 
"ax/5/80 ", 
"ac/5/80 ", 
"ax/6/80 ", 
"ax/2.4/20 ", 
"ax/5/20 ", 
"ac/5/20 ", 
"ax/6/20 ",
"n/2.4/20 ",
]

configs = [
    "ax_160mhz_6ghz",
    "ax_80mhz_5ghz",
    "ac_80mhz_5ghz",
    "ax_80mhz_6ghz",
    "ax_20mhz_24ghz",
    "ax_20mhz_5ghz",
    "ac_20mhz_5ghz",
    "ax_20mhz_6ghz",
    "n_20mhz_24ghz",
]
# formatting

line_styles= [
     'solid',
     'dashed',
     'solid',
     'solid',
     'dashed',
     'solid',
     'dashed',
     'solid',
     'solid',
    ]
marker_styles= [
     '.',
     '.',
     '+',
     'x',
     '_',
     'x',
     '.',
     'o',
     's',
    ]

colors= [
     'maroon',
     'indianred',
     'royalblue',
     'navy',
     'tab:orange',
     'forestgreen',
     'dimgrey',
     'tab:red',
     'tab:orange',
    ]
loc_names= [
    'Short range LoS',
    'Medium range LoS',
    'Long range NLoS',
    'Long range with metallic obstructions',
]

added_var_style = {"linestyle": "dotted", "color": "k", "linewidth":3, "marker":None}

FONT_SIZE = 18
LEGEND_LOC = (0.65, 0.95)
#============================ Global variables =================================

# main data container

data = {}

#============================ Functions ========================================

# ------------------------------------------------------------------------------
# align
## Input: n data lists
## Output: n data lists
# looks at the smalles size data
# trims out the extrs elements in lists

# ------------------------------------------------------------------------------
# subsample
## Input: data list, sample ratio n
## Output: data list
# keeps every n data sample and removes other samples

# ------------------------------------------------------------------------------
# clean_list
## Input:  data list
## Output: data list
# goes thrgough the list and removes "no_data", "no_conn" 

def clean_list (data_list):
    
    i=0

    while i< len(data_list):
        if  data_list [i] in ["no_conn", "no_data"]:
            data_list.pop(i)
        else:
            i=i+1
            

    return  data_list

# ------------------------------------------------------------------------------
# clean_list
## Input:  data list
## Output: data list
# goes thrgough list and counts the number of consecutive delays 
# to return a histogram out of it

def consecutive_overdelay (data_list, threshold):
    i=0
    count = 0
    output = []
    while i< len(data_list):
        if  data_list [i] > threshold:
            count = count+1
        elif count >0:
            output.append(count)
            count = 0
        i=i+1
        
    if count >0:
        output.append(count)
    return  output



# ------------------------------------------------------------------------------
# get moving window
## Input: time_series list, data list
## Output: list of moving avarage data, time series


def get_moving_window (data_list, time_series, datetime, window):
    try:
        df = pd.DataFrame({"datetime":datetime, "data": data_list})
        df['datetime'] = pd.to_datetime(df['datetime'])
        df.set_index("datetime")
        res = df.rolling(window, on = "datetime")
        return res
    except Exception as e:
        print(e)
        pdb.set_trace()
    
# ------------------------------------------------------------------------------
# clean_data
## Input: time_series list, data list
## Output: updated time series, data list, highglights
# goes thrgough the list and removes "no_data"/"no_conn"


def clean (time_series, data_list, datetime=None):
    
    i=0
    no_conn_ts  = []
    no_data_ts  = []
    no_conn     = []
    no_data     = []
    
    if (len (data_list)!=0):
        
        last_val    = data_list [0]
        
        if (len (time_series) != len(data_list) ):
            print ("lengths not matching")
            pdb.set_trace()
        
        while i< len(data_list):
            
            if  data_list [i] in ["no_conn"]:
            
                # store last value so you can highlight where the conn was lost
                
                no_conn.append(last_val)
                data_list.pop(i)
                no_conn_ts.append(time_series.pop(i))
                if (datetime!=None):
                    datetime.pop(i)
                    

            elif  data_list [i] in ["no_data"]:
                
                no_data.append(last_val)
                data_list.pop(i)
                no_data_ts.append(time_series.pop(i))
                if (datetime!=None):
                    datetime.pop(i)
            else:
                last_val = data_list[i]
                i=i+1
            

    highlights = {
        "no_conn_ts"    : no_conn_ts,
        "no_conn"       : no_conn,
        "no_data_ts"    : no_data_ts,
        "no_data"       : no_data,
    }

    return time_series, data_list, highlights, datetime

def get_rssi_curve (freq,rssi):
    bw = 20
    if freq >5000:
        bw = 80
    elif freq > 6000:
        bw = 160
    r = freq +(bw/2)
    l = freq -(bw/2)
    diff = l-r
    x = np.linspace(r, l, 50)
    y = (-99-float(rssi)) * np.sin((x - r) / diff * np.pi)
    y = -99-y
    
    return x,y

def get_channel_curve (freq,channel):
    bw = 20
    if freq >5000:
        bw = 80
    elif freq > 6000:
        bw = 160
    r = freq +(bw/2)
    l = freq -(bw/2)
    diff = l-r
    x = np.linspace(r, l, 50)
    try:
        y = channel * np.sin((x - r) / diff * np.pi)
    except:
        pdb.set_trace()
    
    return x,y
# reference: absolute or relative
def normalize_list (list, reference = "relative"):
    
    new_list = [0]
    i = 1
    while i< len(list):
        if reference == "absolute":
            diff = list[i]-list[0]
        elif reference == "relative":
            diff = list[i]-list[i-1]
        else: 
            pdb.set_trace()
        if (diff>=0):
            new_list.append(diff)
        else:
            # assume no diff
            new_list.append(0)
        i=i+1
    return new_list

def compute_rate (timeseries, dataseries):
    
    new_list = [0]
    i = 1
    if (len (timeseries)!= len(dataseries)):
        print ("incorrect input")
        pdb.set_trace()
        
    while i< len(dataseries):
        data_delta = dataseries[i]-dataseries[i-1]
        time_delta = timeseries[i]- timeseries[i-1]
        if (data_delta>=0):
            new_list.append(data_delta/time_delta)
        else:
            # assume no diff
            new_list.append(0)
        i=i+1
    return new_list
      
def trim_time (timeseries, dataseries, range, datetime=None,):
    
    new_timeseries = []
    new_dataseries = []
    new_datetimeseries = []
    i = 0
    if (len (timeseries)!= len(dataseries)):
        print ("incorrect input")
        pdb.set_trace()
    
    while i< len(dataseries):
        
        if (timeseries[i]>= range[0] and timeseries[i] <= range[1]):
            new_timeseries.append(timeseries[i])
            new_dataseries.append(dataseries[i])
            if datetime != None:
                new_datetimeseries.append(datetime[i])
    
        i=i+1
    return new_timeseries, new_dataseries, new_datetimeseries  
# ------------------------------------------------------------------------------

# plot global box plots per location

## For each location:
##  For each config:
##      clean
##      choose a variable and append all data in all time in one list
##      add it to variable data [loc] [config]
## make grouped plot. 
## x axis: config, y axis: var distribution

def plot_boxplot (settings):
    var_name = settings ["var_name"]
    timestamp = settings ["timestamp"]
    plt.figure()
    
    figure, axis = plt.subplots(2, 2, figsize =(20,15), dpi=150)
    
    i = 0
    for loc in data.keys():
        
        data_table =[]
        data_labels =[]
        j=0
        for config in configs:
            #pdb.set_trace()
            l = int(format(i, '#04b')[2])
            r = int(format(i, '#04b')[3])
            # print (loc, config)
            sanitized_timeseries, sanitized_data, highlights,dt = clean(\
            data[loc][config][timestamp],\
            data[loc][config][var_name])
            
            if "normalize" in settings.keys():
                if settings ["normalize"] == "rate":
                    sanitized_data= compute_rate(sanitized_timeseries,sanitized_data)
                else:
                    sanitized_data= normalize_list(sanitized_data)
            
            if "subtract" in settings.keys():
                # pdb.set_trace()
                subtract_data = data[loc][config][settings["subtract"]]
                sanitized_subtract_data = clean_list (subtract_data)
                sanitized_subtract_data = normalize_list (sanitized_subtract_data)
                v1  = np.array(sanitized_data)
                v2  = np.array(sanitized_subtract_data)
                res = v1-v2
                sanitized_data = list (res)
            
            if "trim_time" in settings.keys():
                sanitized_timeseries, sanitized_data, dt = trim_time(\
                sanitized_timeseries, sanitized_data, settings["trim_time"])
            
            if "absolute" in settings.keys():
                sanitized_data = [abs(ele) for ele in sanitized_data]
            
            data_table.append(sanitized_data)
            data_labels.append(configs_labels[j])
            j=j+1
        
        try:
            df = pd.DataFrame(data_table, index=data_labels)
        except Exception as e:
            print (e)
            pdb.set_trace()
        
        try:
            df.T.boxplot(vert=False, ax = axis [l,r])
            plt.subplots_adjust(left=0.25)
        except Exception as e:
            print (e)
            pdb.set_trace()

        if (settings["ylimit"]!=None):
            axis[l,r].set_xticks(np.arange(settings["ylimit"][0] ,\
            settings["ylimit"][1],settings["stepy"]))
            axis[l,r].set_xlim(settings["ylimit"])
        if ("multi-ylimit" in settings.keys() and settings["multi-ylimit"]!=None):
            ylimit = settings["multi-ylimit"][i]
            axis[l,r].set_xticks(\
            np.arange(ylimit[0] ,\
            ylimit[1],ylimit[2]))
            axis[l,r].set_xlim([ylimit[0], ylimit[1]])
        axis[l,r].set_xlabel(settings["ylabel"],y=0.08, size = 15)
        axis[l,r].tick_params(which='major', labelsize=15)

        axis[l,r].set_title ("Location: {} {}".format(loc, loc_names[int(loc)-1]), size = 15)
        axis[l,r].grid(True)
        i=i+1
    
    figure.tight_layout(pad=7.0)
    figure.supylabel('Configuration', x=0.05, size = 15)
    plt.savefig("output/box/"+ var_name+"_matrix_box.png",\
    bbox_inches='tight')
    plt.close()
    print ("\t boxplot completed.")

# ------------------------------------------------------------------------------

# plot global timeseries plots per location on moving window average basis

## For each location:
##  For each config:
##      align
##      choose a variable and plot line data[loc] [config] [var]
## append to grouped plot

def plot_timeseries_mw (settings):
    var_name = settings ["var_name"]
    datetime = settings ["datetime"]
    timestamp = settings ["timestamp"]
    plt.figure()
    
    figure, axis = plt.subplots(2, 2, sharex = False, figsize =(30,20), dpi=300)
    
    i = 0
    for loc in data.keys():
        j = 0
        for config in configs:
            #pdb.set_trace()
            l = int(format(i, '#04b')[2])
            r = int(format(i, '#04b')[3])
            
            # print (loc, config)
            if (len( data[loc][config][datetime]) != \
            len( data[loc][config][var_name])):
                pdb.set_trace()
               
            sanitized_timeseries, sanitized_data, highlights, sanitized_datetime = clean(\
            data[loc][config][timestamp],\
            data[loc][config][var_name],
            data[loc][config][datetime])
            
            
            if "trim_time" in settings.keys():
                sanitized_timeseries, sanitized_data , sanitized_datetime= trim_time(\
                data[loc][config][timestamp],
                data[loc][config][var_name], 
                settings["trim_time"], 
                datetime=data[loc][config][datetime],
                )
            res = get_moving_window(\
            sanitized_data, 
            sanitized_timeseries,
            sanitized_datetime,
            '5s',
            )
            
            # pdb.set_trace()
            
            axis[l,r].fill_between(
            sanitized_timeseries,
            res.quantile(0.25)["data"].to_list(),\
            res.quantile(0.75)["data"].to_list(),\
            color = colors [j], alpha=0.2)

            axis[l,r].plot(
            sanitized_timeseries,
            res.median()["data"].to_list(),\
            linewidth=1.5, label = configs_labels[j],
            linestyle= line_styles[j], marker = marker_styles[j],\
            color = colors [j])

            j =j+1
        axis[l,r].set_title ("Location: {}".format(loc), size = 15)
        axis[l,r].set_ylabel(settings ["ylabel"])
        axis[l,r].set_xlabel(settings ["xlabel"])
        axis[l,r].legend()
        axis[l,r].grid(True)
        
        if (settings ["ylimit"]!= None and settings ["stepy"] !=None):
            axis[l,r].set_yticks( np.arange(settings ["ylimit"][0],\
            settings ["ylimit"][1],
            settings ["stepy"]))
            axis[l,r].set_ylim (settings ["ylimit"])
        
        if ("multi-ylimit" in settings.keys() and settings["multi-ylimit"]!=None):
            ylimit = settings["multi-ylimit"][i]
            axis[l,r].set_yticks(\
            np.arange(ylimit[0] ,\
            ylimit[1],ylimit[2]))
            axis[l,r].set_ylim([ylimit[0], ylimit[1]])
        
        if ("xlimit" in settings.keys() and settings ["stepx"] !=None):
            axis[l,r].set_xticks( np.arange(settings ["xlimit"][0],\
            settings ["xlimit"][1],
            settings ["stepx"]))
            axis[l,r].set_xlim (settings ["xlimit"])
        i =i+1
    
    #plt.tight_layout()
    plt.savefig("output/time/"+ var_name+"_matrix_sma.png",\
    bbox_inches='tight')
    plt.close()
    print ("\t sma timeseries plot completed.")


# ------------------------------------------------------------------------------

# plot global timeseries plots per location on moving window average basis

## For each location:
##  For each config:
##      align
##      choose a variable and plot line data[loc] [config] [var]
## append to grouped plot

def plot_timeseries_mw_individual (settings, ax = None, fig=None, multiplot = False, set_loc =None):
    var_name = settings ["var_name"]
    datetime = settings ["datetime"]
    timestamp = settings ["timestamp"]
    i = 0
    loc_list = data.keys()
    if not( set_loc is None):
        loc_list=[set_loc]
        
    for loc in loc_list:
        j = 0
        figure = fig
        axis   = ax
        if (ax is None):
            plt.figure()
            figure, axis = plt.subplots(len(configs), sharex = True, sharey = True,  
            figsize =(7,11), 
            dpi=200)
        
        for config in configs:
            print (var_name, loc, config)
            #pdb.set_trace()

            # print (loc, config)
            if (len( data[loc][config][datetime]) != \
            len( data[loc][config][var_name])):
                pdb.set_trace()
               
            sanitized_timeseries, sanitized_data, highlights,sanitized_datetime = clean(\
            data[loc][config][timestamp],\
            data[loc][config][var_name],
            datetime = data[loc][config][datetime])
            
            
            if "trim_time" in settings.keys():
                sanitized_timeseries, sanitized_data , sanitized_datetime= trim_time(\
                sanitized_timeseries,
                sanitized_data, 
                settings["trim_time"], 
                datetime=sanitized_datetime,
                )
            
            
            res = get_moving_window(\
            sanitized_data, 
            sanitized_timeseries,
            sanitized_datetime,
            '5s',
            )            
            
            mylinestyle = line_styles[j]
            mymarker    = marker_styles[j]
            mycolor     = colors [j]
            mylinewidth   = 1.5
            label       = configs_labels[j]

            if "add_var" not in settings.keys():
                mylinestyle = added_var_style["linestyle"]
                mymarker    = added_var_style["marker"]
                mycolor     = added_var_style["color"]
                mylinewidth = added_var_style["linewidth"]
                
            if multiplot :
                label = settings["legend"]
            
            # pdb.set_trace()
            
            axis[j].fill_between(
            sanitized_timeseries,
            # res.quantile(0.1)["data"].to_list(),\
            # res.quantile(0.9)["data"].to_list(),\
            res.min()["data"].to_list(),
            res.max()["data"].to_list(),
            color = mycolor, alpha=0.2)
            
            axis[j].plot(
            sanitized_timeseries,
            res.median()["data"].to_list(),\
            label = label,
            linewidth=mylinewidth,linestyle= mylinestyle, marker =mymarker,\
            color = mycolor)
            axis[j].legend(loc= "upper left")
            axis[j].grid(True)
            
            if (settings ["ylimit"]!= None and settings ["stepy"] !=None):
                axis[j].set_yticks( np.arange(settings ["ylimit"][0],\
                settings ["ylimit"][1],
                settings ["stepy"]))
                axis[j].set_ylim (settings ["ylimit"])
            
            if ("multi-ylimit" in settings.keys() and settings["multi-ylimit"]!=None):
                ylimit = settings["multi-ylimit"][i]
                axis[j].set_yticks(\
                np.arange(ylimit[0] ,\
                ylimit[1],ylimit[2]))
                axis[j].set_ylim([ylimit[0], ylimit[1]])
            
            if ("xlimit" in settings.keys() and settings ["stepx"] !=None):
                axis[j].set_xticks( np.arange(settings ["xlimit"][0],\
                settings ["xlimit"][1],
                settings ["stepx"]))
                axis[j].set_xlim (settings ["xlimit"])
            
            if "add_var" in settings.keys():
                axis[j].set_title (configs_labels[j], y = 0.7)

                
            j =j+1
        i =i+1
        
        if "add_var" in settings.keys():
            var_settings = get_setting(settings["add_var"])
            try:
                axis,figure = plot_timeseries_mw_individual(var_settings,axis,figure, multiplot=True, set_loc = loc)
            except Exception as e:
                print(e)
                pdb.set_trace()
        else:
            return axis, figure
            
        figure.suptitle("Location "+loc, y=0.9)
        figure.supylabel(settings ["ylabel"])
        plt.xlabel(settings ["xlabel"])
        plt.savefig("output/time/{}_{}_matrix_sma.png".format(var_name,loc),\
        bbox_inches='tight')
        plt.close()
        print ("\t sma timeseries plot completed.")
    
    #plt.tight_layout()

# ------------------------------------------------------------------------------

# plot global timeseries plots per location
# input var name

## For each location:
##  For each config:
##      align
##      choose a variable and plot line data[loc] [config] [var]
## append to grouped plot

def plot_timeseries (settings):
    var_name = settings ["var_name"]
    timestamp = settings ["timestamp"]
    plt.figure()
    
    plt.rcParams.update({'font.size': FONT_SIZE})
    figure, axis = plt.subplots(2, 2, sharex = False, figsize =(30,20), dpi=300)

    i = 0
    for loc in data.keys():
        j = 0
        for config in configs:
            #pdb.set_trace()
            l = int(format(i, '#04b')[2])
            r = int(format(i, '#04b')[3])
            
            # print (loc, config)
            if (len( data[loc][config][timestamp]) != \
            len( data[loc][config][var_name])):
                pdb.set_trace()
               
            sanitized_timeseries, sanitized_data, highlights,dt = clean(\
            data[loc][config][timestamp],\
            data[loc][config][var_name])
            
            if "normalize" in settings.keys():
                if settings ["normalize"] == "rate":
                    sanitized_data= compute_rate(sanitized_timeseries,sanitized_data)
                else:
                    sanitized_data= normalize_list(sanitized_data)
            
            if "subtract" in settings.keys():
                # pdb.set_trace()
                subtract_data = data[loc][config][settings["subtract"]]
                sanitized_subtract_data = clean_list (subtract_data)
                sanitized_subtract_data = normalize_list (sanitized_subtract_data)
                v1  = np.array(sanitized_data)
                v2  = np.array(sanitized_subtract_data)
                res = v1-v2
                sanitized_data = list (res)
               
            
            if "trim_time" in settings.keys():
                sanitized_timeseries, sanitized_data,dt = trim_time(\
                sanitized_timeseries, sanitized_data, settings["trim_time"])
            
            axis[l,r].plot(
            sanitized_timeseries,\
            sanitized_data,\
            linewidth=1.5, label = configs_labels[j],
            linestyle= line_styles[j], marker = marker_styles[j],\
            color = colors [j])

            j =j+1
        axis[l,r].set_title ("Location: {} {}".format(loc,loc_names[int(loc)-1]))
        axis[l,r].set_ylabel(settings ["ylabel"])
        axis[l,r].set_xlabel(settings ["xlabel"])
        axis[l,r].grid(True)

        if ("ylimit" in settings.keys() and "stepy" in settings.keys()):
            axis[l,r].set_yticks( np.arange(settings ["ylimit"][0],\
            settings ["ylimit"][1],
            settings ["stepy"]))
            axis[l,r].set_ylim (settings ["ylimit"])
        
        if ("xlimit" in settings.keys() and settings ["stepx"] !=None):
            axis[l,r].set_xticks( np.arange(settings ["xlimit"][0],\
            settings ["xlimit"][1],
            settings ["stepx"]))
            axis[l,r].set_xlim (settings ["xlimit"])
        
        # if ("legend_loc" in settings.keys()):
            # axis[l,r].legend(loc=settings["legend_loc"])
        # else:
            # axis[l,r].legend()
            
        if ("multi-ylimit" in settings.keys() and settings["multi-ylimit"]!=None):
            ylimit = settings["multi-ylimit"][i]
            axis[l,r].set_yticks(\
            np.arange(ylimit[0] ,\
            ylimit[1],ylimit[2]))
            axis[l,r].set_ylim([ylimit[0], ylimit[1]])
        if i==2:
            handles, labels = axis[l,r].get_legend_handles_labels()
        i=i+1
    
    figure.legend(handles, labels, loc='right', bbox_to_anchor=LEGEND_LOC, ncols = 3)
    #plt.tight_layout()
    plt.savefig("output/time/"+ var_name+"_matrix_time.png",\
    bbox_inches='tight')
    plt.close()
    print ("\t timeseries plot completed.")

# ------------------------------------------------------------------------------

# plot global scatter plots per location
# input var name

## For each location:
##  For each config:
##      align
##      choose a variable and plot scatter data[loc] [config] [var]
##       add any highlights as axvline
## append to grouped plot

def plot_scatter (settings):
    var_name = settings ["var_name"]
    timestamp = settings ["timestamp"]
    plt.figure()
    plt.rcParams.update({'font.size': FONT_SIZE})
    figure, axis = plt.subplots(2, 2, sharex = False, figsize =(30,20), dpi=300)
    
    i = 0
    for loc in data.keys():
        j = 0
        for config in configs:
            #pdb.set_trace()
            l = int(format(i, '#04b')[2])
            r = int(format(i, '#04b')[3])
            
            # print (loc, config)
            if (len( data[loc][config][timestamp]) != \
            len( data[loc][config][var_name])):
                pdb.set_trace()
                
            sanitized_timeseries, sanitized_data, highlights,dt = clean(\
            data[loc][config][timestamp],\
            data[loc][config][var_name])
            if "normalize" in settings.keys():
                if settings ["normalize"] == "rate":
                    sanitized_data= compute_rate(sanitized_timeseries,sanitized_data)
                else:
                    sanitized_data= normalize_list(sanitized_data)
            
            if "subtract" in settings.keys():
                # pdb.set_trace()
                subtract_data = data[loc][config][settings["subtract"]]
                sanitized_subtract_data = clean_list (subtract_data)
                sanitized_subtract_data = normalize_list (sanitized_subtract_data)
                v1  = np.array(sanitized_data)
                v2  = np.array(sanitized_subtract_data)
                res = v1-v2
                sanitized_data = list (res)
            
            if "trim_time" in settings.keys():
                sanitized_timeseries, sanitized_data, dt = trim_time(\
                sanitized_timeseries, sanitized_data, settings["trim_time"])    
            axis[l,r].scatter(
            sanitized_timeseries,\
            sanitized_data,\
            linewidth=1.5, label = configs_labels[j],
            linestyle= line_styles[j], marker = marker_styles[j],\
            color = colors [j],
            alpha = 0.5)
            
            # if highlights!=None:
                # k = 0
                
                # while k < len (highlights["no_conn"]):
                    # axis[l,r].axvline(x =highlights["no_conn_ts"][k] , \
                    
                    # color = colors [j],
                    # alpha = 0.5)
                    # k=k+1
                
                # k = 0
                # while k < len (highlights["no_data_ts"]):
                    # axis[l,r].axvline(x =highlights["no_data_ts"][k] , \
                    # # ymin = highlights["no_conn"][k]*,  
                    # # ymax = highlights["no_conn"][k]* , 
                    # color = colors [j],
                    # alpha = 0.5)
                    # k=k+1
            
            
            j =j+1
        axis[l,r].set_title ("Location: {} {}".format(loc, loc_names[int(loc)-1]))
        axis[l,r].set_xlabel(settings["ylabel"],y=0.08)
        axis[l,r].set_xlabel(settings ["xlabel"])
        # axis[l,r].legend()

        axis[l,r].grid(True)
        
        if ("ylimit" in settings.keys() and "stepy" in settings.keys()):
            axis[l,r].set_yticks( np.arange(settings ["ylimit"][0],\
            settings ["ylimit"][1],
            settings ["stepy"]))
            axis[l,r].set_ylim (settings ["ylimit"])
            
        if ("xlimit" in settings.keys() and settings ["stepx"] !=None):
            
            axis[l,r].set_xticks( np.arange(settings ["xlimit"][0],\
            settings ["xlimit"][1],
            settings ["stepx"]))
            axis[l,r].set_xlim (settings ["xlimit"])
        
        if ("multi-ylimit" in settings.keys() and settings["multi-ylimit"]!=None):
            ylimit = settings["multi-ylimit"][i]
            axis[l,r].set_yticks(\
            np.arange(ylimit[0] ,\
            ylimit[1],ylimit[2]))
            axis[l,r].set_ylim([ylimit[0], ylimit[1]])
        if i==2:
            handles, labels = axis[l,r].get_legend_handles_labels()
        
        i=i+1
    
    figure.legend(handles, labels, loc='right', bbox_to_anchor=LEGEND_LOC, ncols = 3)
    plt.savefig("output/scatter/"+ var_name+"_matrix_scatter.png",\
    bbox_inches='tight')
    plt.close()
    print ("\t scatter plot completed.")
    
# ------------------------------------------------------------------------------

# plot 2D histogram
# input datalist

## For each location:
##  For each config:
##      align
##      choose a variable and plot historgram
## append to grouped plot

def plot_hist2d (settings):

    var_name = settings ["var_name"]
    timestamp = settings ["timestamp"]
    plt.figure()
    mybins = [1,2,3,4,5,6]
    figure, axis = plt.subplots(1, 4, figsize =(15,5.5), dpi=300)
    plt.rcParams.update({'font.size': FONT_SIZE})
    hist2d_xlabel = settings["ylabel"]
    i = 0
    
    for loc in data.keys():
        j = 0
        loc_data = []
        for config in configs:
            
            sanitized_timeseries, sanitized_data, highlights, dt= clean(\
            data[loc][config][timestamp],\
            data[loc][config][var_name])
            
            
            if "normalize" in settings.keys():
                if settings ["normalize"] == "rate":
                    sanitized_data= compute_rate(sanitized_timeseries,sanitized_data)
                else:
                    sanitized_data= normalize_list(sanitized_data)
            
            if "subtract" in settings.keys():
                # pdb.set_trace()
                subtract_data = data[loc][config][settings["subtract"]]
                sanitized_subtract_data = clean_list (subtract_data)
                sanitized_subtract_data = normalize_list (sanitized_subtract_data)
                v1  = np.array(sanitized_data)
                v2  = np.array(sanitized_subtract_data)
                res = v1-v2
                sanitized_data = list (res)
            
            if "trim_time" in settings.keys():
                sanitized_timeseries, sanitized_data, datetime = trim_time(\
                sanitized_timeseries, sanitized_data, settings["trim_time"])
            
            # pdr =  len(sanitized_data)/120
            overdelayed = 0
            if len(sanitized_data) >0:
                x= consecutive_overdelay(sanitized_data,75)
                count, bins_count = np.histogram (x, bins=mybins)
                loc_data.append(list(count))
            else:
                loc_data.append(list(np.ones(5, dtype = int)*0))
                
            j=j+1
            
            
        if "hist2d_xlabel" in settings.keys():
            hist2d_xlabel = settings["hist2d_xlabel"]

        pos = axis[i].imshow(loc_data, cmap  = "coolwarm", vmin =0, vmax = 14)
        axis[i].set_title ("Location: {}".format(loc))
        axis[i].set_xticks(np.arange(5), labels = mybins[0:-1], size= FONT_SIZE)
        
        if i == 0:
            axis[i].set_yticks(np.arange(len(configs_labels)),labels = configs_labels, size= FONT_SIZE)
        else:
            axis[i].get_yaxis().set_visible(False)
            
        i=i+1

    figure.supxlabel(hist2d_xlabel)
    figure.subplots_adjust(right=0.8)
    cbar_ax = figure.add_axes([0.82, 0.12, 0.01, 0.8])
    cb = figure.colorbar(pos, cax=cbar_ax)
    cb.set_label("number of occurences")
    
    plt.savefig("output/2d/overdelay_hist.png",\
    bbox_inches='tight')
    plt.close()
    print ("\t hist2d plot completed.")



# ------------------------------------------------------------------------------

# plot global CDF plots per location
# input var name

## For each location:
##  For each config:
##      align
##      choose a variable and create CDF
##      plot line data[loc] [config] [var]
## append to grouped plot

def plot_cdf (settings):

    var_name = settings ["var_name"]
    timestamp = settings ["timestamp"]
    plt.figure()
    plt.rcParams.update({'font.size': FONT_SIZE})
    figure, axis = plt.subplots(2, 2, figsize =(25,20), dpi=300)
    
    i = 0
    for loc in data.keys():
        j = 0
        for config in configs:
            l = int(format(i, '#04b')[2])
            r = int(format(i, '#04b')[3])
            
            sanitized_timeseries, sanitized_data, highlights, dt= clean(\
            data[loc][config][timestamp],\
            data[loc][config][var_name])
            
            
            if "normalize" in settings.keys():
                if settings ["normalize"] == "rate":
                    sanitized_data= compute_rate(sanitized_timeseries,sanitized_data)
                else:
                    sanitized_data= normalize_list(sanitized_data)
            
            if "subtract" in settings.keys():
                # pdb.set_trace()
                subtract_data = data[loc][config][settings["subtract"]]
                sanitized_subtract_data = clean_list (subtract_data)
                sanitized_subtract_data = normalize_list (sanitized_subtract_data)
                v1  = np.array(sanitized_data)
                v2  = np.array(sanitized_subtract_data)
                res = v1-v2
                sanitized_data = list (res)
            
            if "trim_time" in settings.keys():
                sanitized_timeseries, sanitized_data, datetime = trim_time(\
                sanitized_timeseries, sanitized_data, settings["trim_time"])
            
            pdr =  len(sanitized_data)/120
            if pdr>1:
                pdr = 1
            overdelayed = 0
            if len(sanitized_data) >0:
                overdelayed = sum (i < 75 for i in sanitized_data)/len(sanitized_data)
                # print ("{},{},{}".format(loc,config, mean(sanitized_data)))
            # else:
                # print ("{},{},{}".format(loc,config, -1))
            # print (config,loc, overdelayed * pdr)
            # print ("{},{},{}".format(loc,config, pdr*overdelayed))
            # print ("{},{},{}".format(loc,config, max(sanitized_data)))
            count, bins_count = np.histogram (sanitized_data, bins=30)
            pdf = count / sum(count)
            cdf = np.cumsum(pdf)
            axis[l,r].plot(bins_count[1:],cdf,\
                label = configs_labels[j],\
                linestyle=\
                    line_styles[j],\
                    marker = marker_styles[j],\
                    color = colors [j]
                )
            j=j+1

        if ("ylimit" in settings.keys() and "stepy" in settings.keys()):
            #pdb.set_trace()
            axis[l,r].set_xticks(\
            np.arange(settings["ylimit"][0] ,\
            settings["ylimit"][1],settings["stepy"]))
            axis[l,r].set_xlim(settings["ylimit"])
        
        if ("multi-ylimit" in settings.keys() and settings["multi-ylimit"]!=None):
            ylimit = settings["multi-ylimit"][i]
            axis[l,r].set_xticks(\
            np.arange(ylimit[0] ,\
            ylimit[1],ylimit[2]))
            axis[l,r].set_xlim([ylimit[0], ylimit[1]])
                  
    
        axis[l,r].set_yticks(np.arange(0,1.1,0.1))
        axis[l,r].set_title ("Location: {} {}".format(loc, loc_names[int(loc)-1]))
        axis[l,r].grid(True)
        axis[l,r].set_ylabel('CDF')
        axis[l,r].set_xlabel(settings["ylabel"],y=0.08)
        if i==2:
            handles, labels = axis[l,r].get_legend_handles_labels()
        i=i+1
    
    figure.legend(handles, labels, loc='right', bbox_to_anchor=LEGEND_LOC, ncols = 3)
    plt.savefig("output/cdf/"+ var_name+"_matrix_cdf.png",\
    bbox_inches='tight')
    plt.close()
    print ("\t CDF plot completed.")

# ------------------------------------------------------------------------------

# plot global 3D plots per location
# input: X, Y, Z var names, subsample_var (X,Y, or Z), subsample_ratio 

## For each location:
##  For each config:
##      subsample if needed
##      align
##      Assign X, y, z data to lines
##      plot 3D
## append to grouped plot


# ------------------------------------------------------------------------------

# plot AP info
# input: band [2G, 5G, 6G]), plot settings
# output: curve plot of the rssis of aps at each frequencey

## For each location:
##  For each config:
##   pick one scan is enought
##   for each ap, plot curve
## append to grouped plot

def plot_ap_rssi_scan (band,settings):

    i = 0
    for loc in data.keys():
        
        for config in data[loc].keys():
            number_of_scans = len(data[loc][config]["apscan_"+band])
            if (number_of_scans==0):
                continue  
            scan_id = 0
            while scan_id<number_of_scans:
                freq = clean_list(data[loc][config]\
                ["apscan_"+band][scan_id]["apscan_freq"])
                
                if (len(freq)>0):
                    rssi = clean_list(data[loc][config]\
                    ["apscan_"+band][scan_id]["apscan_rssi"])
                    ssid = clean_list(data[loc][config]\
                    ["apscan_"+band][scan_id]["apscan_ssid"])
                    channel_use = clean_list(data[loc][config]\
                    ["apscan_"+band][scan_id]["apscan_channel_utilisation"])

                    plt.figure()            
                    #figure,axis  =  plt.subplots(2, 2, figsize =(25,20), dpi=300)
                    figure,axis  =  plt.subplots(figsize =(8,2), dpi=200)
                    
                    k = 0
                    while k< len(freq):
                        if channel_use[k]==[]:
                            pdb.set_trace()
                        x,y = get_rssi_curve(freq[k],rssi[k])
                        colors = ["r","g","b"]
                        if ("INSA_ASUS"in ssid[k]):
                            axis.plot(x,y, color="b")
                            axis.fill(x, y, color="b", alpha=0.3)
                        else:
                            axis.plot(x,y, color="g")
                            axis.plot(x,y, color="g", alpha=0.3)
                        k=k+1

                        axis.set_title ("RSSI location: {}".\
                            format(loc))
                        axis.grid(True)
                        axis.set_xticks(settings ["xticks"])
                        axis.set_yticks(settings ["yticks"])
                        axis.set_ylabel('RSSI (dBm)')
                        axis.set_xlabel("Frequency (MHz)")
                        axis.set_ylim(settings["ylim"])
                    print ("saving: ",band,loc,config)        
                    plt.savefig("output/ap_scan/single_rssi_scans/ap_scan_rssi_{}_{}_{}_#{}.png"\
                    .format(band,loc,config,scan_id),\
                    bbox_inches='tight')
                    plt.close()
                    
                    scan_id= scan_id+1
                else:
                    continue
        i=i+1
    print ("\t ap scan rssi plot completed for ", band, " band")


def plot_ap_channel_scan (band,settings):

    i = 0
    for loc in data.keys():
        
        for config in data[loc].keys():
            number_of_scans = len(data[loc][config]["apscan_"+band])
            if (number_of_scans==0):
                continue  
            scan_id = 0
            while scan_id<number_of_scans:
                freq = clean_list(data[loc][config]\
                ["apscan_"+band][scan_id]["apscan_freq"])
                
                if (len(freq)>0):
                    rssi = clean_list(data[loc][config]\
                    ["apscan_"+band][scan_id]["apscan_rssi"])
                    ssid = clean_list(data[loc][config]\
                    ["apscan_"+band][scan_id]["apscan_ssid"])
                    channel_use = clean_list(data[loc][config]\
                    ["apscan_"+band][scan_id]["apscan_channel_utilisation"])

                    plt.figure()            
                    figure,axis  =  plt.subplots(figsize =(8,2), dpi=200)
                    
                    k = 0
                    while k< len(freq):
                        if channel_use[k]==[]:
                            pdb.set_trace()
                        x,y = get_channel_curve(freq[k],channel_use[k])
                        colors = ["r","g","b"]
                        if ("INSA_ASUS"in ssid[k]):
                            axis.plot(x,y, color="b")
                            axis.fill(x, y, color="b", alpha=0.3)
                        else:
                            axis.plot(x,y, color="g")
                            axis.plot(x,y, color="g", alpha=0.3)
                        k=k+1

                        axis.set_title ("Channel utilization location: {}".\
                            format(loc))
                        axis.grid(True)
                        axis.set_xticks(settings ["xticks"])
                        axis.set_yticks(settings ["yticks"])
                        axis.set_ylabel('Channel Utilization (%)')
                        axis.set_xlabel("Frequency (MHz)")
                        axis.set_ylim(settings["ylim"])
                    print ("saving: ",band,loc,config)        
                    plt.savefig("output/ap_scan/single_chu_scans/ap_scan_chu_{}_{}_{}_#{}.png"\
                    .format(band,loc,config,scan_id),\
                    bbox_inches='tight')
                    plt.close()
                    
                    scan_id= scan_id+1
                else:
                    continue
        i=i+1
    print ("\t ap scan channel use plot completed for ", band, " band")

# ------------------------------------------------------------------------------

# plot AP info per experiment run
# input: band [2G, 5G, 6G]), plot settings
# output: curve plot of the rssis of aps at each frequencey

## For each location:
##  For each config:
##   pick one scan is enought
##   for each ap, plot curve
## append to grouped plot

def plot_ap_scan_individual (band,settings):

    plt.figure(figsize =(25,20), dpi=300)
       
    i = 0
    for loc in data.keys():
        j = 0
        found_scan = False
        for config in configs:
            freq = []
            
            n_captutres = len (data[loc][config]["apscan_"+band])
            if  n_captutres == 0 :
                continue
            
            a = 0
            while (a<n_captutres):
                freq= clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_freq"])
                rssi = clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_rssi"])
                ssid = clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_ssid"])
                sta_count = clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_station_count"])
                channel_use = clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_channel_utilisation"])
                df = pd.DataFrame(list(zip(ssid,rssi,freq,channel_use,sta_count)),\
                columns =['SSID', 'RSSI', "Freq", "channel use","STA count" ])
                print ("location", loc, config,"capture", a)
                # cmap = sn.light_palette("green", as_cmap=True)
                # df = df.style.background_gradient(cmap=cmap)
                print(df.to_string())

                
                k = 0
                while k< len(freq):
                    x,y = get_rssi_curve(freq[k],rssi[k])
                    colors = ["r","g","b"]
                    if ("INSA_ASUS"in ssid[k]):
                        plt.plot(x,y, color="b")
                        plt.fill(x, y, color="b", alpha=0.3)
                    else:
                        plt.plot(x,y, color="g")
                        plt.plot(x,y, color="g", alpha=0.3)
                    k=k+1
    
                plt.title ("RSSI location: {}".\
                    format(loc))
                plt.grid(True)
                plt.xticks(settings ["xticks"])
                plt.yticks(settings ["yticks"])
                plt.ylabel('RSSI (dBm)')
                plt.xlabel("Frequency (MHz)")
                plt.ylim(settings["ylim"])
                plt.savefig("output/ap_scan/single_ap_scan_{}_{}_{}_{}.png"\
                .format(band,loc,config,a), bbox_inches='tight')
                plt.close()
                    
                a = a+1
            else:
                continue
            j=j+1
        i=i+1
        print ("\t indvidiaul ap scans completed for ", band, " band")
# ------------------------------------------------------------------------------

# plot RF stats for locations
# input: band [2G, 5G, 6G]), plot settings
# output: curve plot of the rssis of aps at each frequencey


def plot_location_rf_stats (band,settings):

    plt.figure(figsize =(25,20), dpi=300)
       
    i = 0
    
    for loc in data.keys():
        freq_all = []
        ssid_all = []
        rssi_all = []
        sta_count_all = []
        channel_use_all = []
        j = 0
        found_scan = False
        for config in configs:
            freq = []
            n_captutres = len (data[loc][config]["apscan_"+band])
            if  n_captutres == 0 :
                continue
            
            a = 0
            if (a<n_captutres):
                freq_all.extend(clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_freq"]))
                ssid_all.extend(clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_ssid"]))
                rssi_all.extend(clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_rssi"]))
                sta_count_all.extend(clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_station_count"]))
                channel_use_all.extend(clean_list(data[loc][config]\
                ["apscan_"+band][a]["apscan_channel_utilisation"]))
   
                # k = 0
                # while k< len(freq):
                    # x,y = get_rssi_curve(freq[k],rssi[k])
                    # colors = ["r","g","b"]
                    # if ("INSA_ASUS"in ssid[k]):
                        # plt.plot(x,y, color="b")
                        # plt.fill(x, y, color="b", alpha=0.3)
                    # else:
                        # plt.plot(x,y, color="g")
                        # plt.plot(x,y, color="g", alpha=0.3)
                    # k=k+1

                # plt.title ("RSSI location: {}".\
                    # format(loc))
                # plt.grid(True)
                # plt.xticks(settings ["xticks"])
                # plt.yticks(settings ["yticks"])
                # plt.ylabel('RSSI (dBm)')
                # plt.xlabel("Frequency (MHz)")
                # plt.ylim(settings["ylim"])
                # plt.savefig("output/ap_scan/single_ap_scan_{}_{}_{}_{}.png"\
                # .format(band,loc,config,a), bbox_inches='tight')
                # plt.close()
                    
                a = a+1
            else:
                continue
            j=j+1
        
        df = pd.DataFrame(list(zip(ssid_all,rssi_all,freq_all,channel_use_all,sta_count_all)),\
        columns =['SSID', 'RSSI', "Freq", "channel use","STA count" ])
        print("location", loc, df.to_string())
                
        i=i+1
        print ("\t indvidiaul ap scans completed for ", band, " band")            
# =========================== main program =====================================
# load data

f = open("perama_range_testing.json")

line = f.readline()
data = json.loads(line)

settings_list = [
    {
       "var_name"   : "control_delay_ms",
       "timestamp"  : "control_timestamp_s",
       "datetime"   : "control_datetime",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Delay (ms)",
       "legend"     : "ROS delay",
       "hist2d_xlabel"     : "successive overdelay",
       "stepy"      : 25,
       "ylimit"     : [0,101],
       "stepx"      : 10,
       "xlimit"     : [120,181],
       "trim_time"  : [120,181],
       # "multi-ylimit":[[-40,151,20], [-40,151,20], [-90,-74,5], [-90,-59,5]],
       "add_var"    : "ptp_path_delay_current_ms"

    },
    {
       "var_name"   : "control_loss",
       "timestamp"  : "control_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Loss (packets)",

    },
    {
       "var_name"   : "ptp_followups_received",
       "timestamp"  : "ptp_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Received PTP Followups/s",

       "normalize"  : "rate",
    },
    {
       "var_name"   : "ptp_delay_req_sent",
       "timestamp"  : "ptp_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "requests/s",
       "subtract"   : "ptp_delay_req_received",
       #"normalize"  : "rate",
       "subtract"   : "ptp_delay_resp_received",
    },
    {
       "var_name"   : "phy_rssi",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "RSSI (dBm)",
       "multi-ylimit":[[-75,-49,5], [-75,-49,5], [-90,-74,5], [-90,-59,5]]
    },
    {
       "var_name"   : "phy_tx_bitrate",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Tx bitrate (Mbps)",
       "legend_loc"     : "upper right",
       "multi-ylimit":[[0,1301,200], [0,901,200], [0,201,20], [0,400,50]]
    },
    {
       "var_name"   : "phy_tx_packets",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Packets",
    },
    {
       "var_name"   : "phy_rx_packets",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Packets",
    },
    {
       "var_name"   : "phy_tx_mb",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Network interface transmission rate (Mbps)",
       "normalize"  : "rate"
    },
    {
       "var_name"   : "phy_rx_mb",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Network interface receive rate (Mbps)",
       "normalize"  : "rate"
    },
    {
       "var_name"   : "phy_rx_bitrate",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Rx bitrate (Mbps)",
       "trim_time"  : [30,185]
    },
    {
       "var_name"   : "phy_tx_nss",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "NSS",
    },
    {
       "var_name"   : "phy_rx_nss",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Rx NSS",
    },
    {
       "var_name"   : "phy_tx_mcs",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "TX MCS",
    },
    {
       "var_name"   : "phy_rx_mcs",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "RX MCS",
    },
    {
       "var_name"   : "phy_freq",
       "timestamp"  : "phy_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "freq",

    },
    {
       "var_name"   : "iperf_mbps",
       "timestamp"  : "iperf_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "Throughput (Mbps)",
       # "stepy"      : 50,
       # "ylimit"     : [0,851],
       "stepx"      : 20,
       "xlimit"     : [0,181],
       "trim_time"  : [0,181],
    },
    {
       "var_name"   : "iperf_retransmits",
       "timestamp"  : "iperf_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "TCP Retransmits",
       "multi-ylimit": [[0,2.1,1],[0,2.1,1],[0,12.1,1],[0,5.1,1]]
       
    },
    {
       "var_name"   : "iperf_snd_cwnd",
       "timestamp"  : "iperf_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "TCP congestion window",
    },
    {
       "var_name"   : "iperf_rtt",
       "timestamp"  : "iperf_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "TCP RTT (ms)",
       "xlimit"     : [120,181],
       "stepx"      : 10,
       # "trim_time"  : [120,181],
    },
    {
       "var_name"   : "ptp_path_delay_current_ms",
       "timestamp"  : "ptp_timestamp_s",
       "datetime"   : "ptp_datetime",
       "xlabel"     : "Time (s)",
       "ylabel"     : "PTP path delay (ms)",
       "legend"     : "PTP delay",
       "stepy"      : 25,
       "ylimit"     : [0,101],
       "stepx"      : 10,
       "xlimit"     : [120,181],
       "trim_time"  : [120,181],
    },
    {
       "var_name"   : "ptp_path_delay_mean_ms",
       "timestamp"  : "ptp_timestamp_s",
       "xlabel"     : "Time (s)",
       "ylabel"     : "PTP mean delay offset (ms)",
       "stepy"      : 5,
       "ylimit"     : [0,30],
       "stepx"      : 20,
       "xlimit"     : [120,185],
       "trim_time"  : [120,185],
    },
    {
       "var_name"   : "ptp_offset_current_ms",
       "timestamp"  : "ptp_timestamp_s",
       "datetime"   : "ptp_datetime",
       "xlabel"     : "Time (s)",
       "ylabel"     : "PTP correction offset absolute value (ms)",
       "stepy"      : 10,
       "ylimit"     : None,
       "multi-ylimit": [[0,41,5],[0,51,5],[0,81,5],[0,41,5]],
       "stepx"      : 5,
       "xlimit"     : [120,181],
       "trim_time"  : [120,181],
       "absolute"   : True
    },

]

def get_setting (s):
    i = 0
    while i < len(settings_list):
        if settings_list [i]["var_name"] == s:
            return settings_list [i]
        i = i +1
    return None


 

for settings in [
    # get_setting("ptp_offset_current_ms"),
    # get_setting("ptp_path_delay_current_ms"),
    get_setting("iperf_rtt"),
    get_setting("phy_tx_bitrate"),
    get_setting("iperf_mbps"),
    get_setting("phy_rssi"),
    ]:
    data = json.loads(line)
    print ("processing: ", settings ["var_name"])
    plot_cdf (settings)
    plot_scatter (settings)
    plot_timeseries (settings)
    # plot_boxplot (settings)
sys.exit()  

data = json.loads(line)
plot_timeseries_mw_individual (get_setting("control_delay_ms"), multiplot=True)
 
plot_hist2d(get_setting("control_delay_ms"))




plot_ap_rssi_scan("2G", {
    "xticks": np.linspace(2400,2480,9),
    "yticks": np.linspace(-100,-40,13),
    "ylim"  : [-99,-41],
    })
plot_ap_rssi_scan("5G", {
    "xticks": np.linspace(5100,5700,7),
    "yticks": np.linspace(-100,-40,13),
    "ylim"  : [-99,-41],
    
    })

plot_ap_channel_scan("2G", {
    "xticks": np.linspace(2400,2480,9),
    "yticks": np.linspace(0,100,11),
    "ylim"  : [0,101],
    })
  
plot_ap_channel_scan("5G", {
    "xticks": np.linspace(5100,5700,7),
    "yticks": np.linspace(0,100,11),
    "ylim"  : [0,101],
    })




plot_ap_rssi_scan("6G", {
    "xticks": np.linspace(6300,6400,10),
    "yticks": np.linspace(-100,-40,13),
    "ylim"  : [-99,-41],
    })


plot_ap_scan_individual("5G", {
    "xticks": np.linspace(5100,5700,7),
    "yticks": np.linspace(-100,-40,13),
    "ylim"  : [-99,-41],
    })






print ("Done.")



sys.exit()