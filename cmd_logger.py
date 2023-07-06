'''
Script to do passive scanning of WiFi interface using NetInfoView tool
It runs the scan for a priod of time and stores it into json format


Bugwright2 Prject

Mina Rady mina.rady@insa-lyon.fr
'''

import logging
import os
import io
import time
import datetime
import subprocess
import sys
from subprocess import check_output
import json

expid = sys.argv[1]
runid = sys.argv[2]
loc   = sys.argv[3]
duration     = int(sys.argv[4])

cmd_name = sys.argv [5]
cmd      = sys.argv [6]
rate     = float (sys.argv[7]) # 0 = execute once only

period = 0.5


USAGE = "Usage: wifiinfo.py <exp ID> <run ID> <loc> <duration> <cmdname> <cmd>"
print (USAGE)

# log settings


LOG_DIR_NAME = 'logs'

log_file_path = ''

def prepare_log_file():
    log_dir_path = os.path.join( "/home/minarady/","wifi_ranger", LOG_DIR_NAME)

    # make sure we have the log directory
    if os.path.isdir(log_dir_path):
        # log directory is ready :-)
        pass
    else:
        try:
            os.mkdir(log_dir_path)
        except OSError as err:
            sys.exit('Failed to make the log directory: {}'.format(err))

    # time.strftime('%Y%m%d-%H%M%S')
    # decide a log file name and create it
    
    log_file_name = 'log_{}_{}_{}_{}.json'.format(expid, runid, loc, cmd_name)
    log_file_path = os.path.join(log_dir_path, log_file_name)
    
    if os.path.exists(log_file_path):
        os.remove(log_file_path)
        msg = (
            'Replacing file.\n' +
            'Log file already exits: {}'.format(log_file_path)
        )
        
    else:
        # create an empty file with the log file name
        try:
            open(log_file_path, 'w').close()
        except OSError as err:
            sys.exit('Failed to create a log file: {}'.format(err))

    return log_file_path


def process_payload(data):
    # pdb.set_trace()
    #clean_data = str(data).replace("\"","").replace("data: ","")
    #data_arr = clean_data.split(",")  
    
    res = {
        cmd_name                 : data,
        "src_timestamp_ns"    : time.time_ns(),
        "expid"               : expid,
        "runid"               : runid,
        "loc"                 : loc,
        
    }
    return res

def log_data (data):
    global log_file_path
    ts = datetime.datetime.now()
    payload_js = process_payload(data)
    with open(log_file_path, 'a') as f:
        log = {
            'datetime': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
            'timestamp': time.time_ns(),
            'payload'  : payload_js
        }
        f.write('{}\n'.format(json.dumps(log)))

log_file_path = prepare_log_file() 

print (log_file_path)
start = time.time()
now = start
finished = False

while (now-start < duration and not finished):
    out= os.popen(cmd).read()
    print (out)
    log_data(out)
    if rate == 0:
        finished = True
    else:
        time.sleep(1/rate)
        now = time.time()
 
print ("cmd done, exiting  ...")
sys.exit()
