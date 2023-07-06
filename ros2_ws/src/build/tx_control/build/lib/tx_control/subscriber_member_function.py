# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time
import datetime
import json
import os
import pdb

USAGE = "Usage: listener <exp ID> <run ID> <loc> <topic> <duration>"
print (USAGE)


#=========================== Initialize =======================================

LOG_DIR_NAME = 'logs'
expid        = sys.argv[1]
runid        = sys.argv[2]
loc          = sys.argv[3]
topic        = sys.argv[4]
duration     = int(sys.argv[5])
start   = 0
now     = 0





class MinimalSubscriber(Node):

#============================ Main Functions ==================================

    def __init__(self, log_dir_path_arg, expid_arg, runid_arg, loc_arg, topic_arg, duration_arg):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_control',
            self.listener_callback,
            10)
        self.start      =time.time()
        self.expid      = expid_arg
        self.runid      = runid_arg 
        self.loc        = loc_arg
        self.topic      = topic_arg
        self.duration   = duration_arg
        self.now        = start
        self.log_dir_path = log_dir_path_arg
        self.log_file_path = self.prepare_log_file()
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.log_data (msg.data)
        self.now = time.time()
        if (self.now-self.start > self.duration):
            sys.exit()

#============================ Helper Functions ================================

    def prepare_log_file(self):
        # make sure we have the log directory
        if os.path.isdir(self.log_dir_path):
            # log directory is ready :-)
            pass
        else:
            try:
                os.mkdir(self.log_dir_path)
            except OSError as err:
                sys.exit('Failed to make the log directory: {}'.format(err))

        # time.strftime('%Y%m%d-%H%M%S')
        # decide a log file name and create it
        
        log_file_name = 'log_{}_{}_{}.json'.format(expid, runid,loc)
        log_file_path = os.path.join(self.log_dir_path, log_file_name)
        pdb.set_trace()
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
        
        print (log_file_path)
        return log_file_path

    def process_payload(self,data):
        # pdb.set_trace()
        clean_data = str(data).replace("\"","").replace("data: ","")
        data_arr = clean_data.split(",")
                
        print("delay", (time.time_ns()-int (data_arr[1]))/10**6, "ms")
        res = {
            "seqnum"              : int (data_arr[0]),
            "src_timestamp_ns"    : data_arr[1],
            "expid"               : data_arr[2],
            "runid"               : data_arr[3],
            "loc_tx"              : data_arr[4],
            "loc_rx"              : self.loc,
            
        }
        return res

    def log_data (self, data):
        ts = datetime.datetime.now()
        payload_js = self.process_payload(data)
        with open(self.log_file_path, 'a') as f:
            log = {
                'datetime': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                'timestamp': time.time_ns(),
                'payload'  : payload_js
            }
            f.write('{}\n'.format(json.dumps(log)))


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

def shutdown_hook():
  sys.exit()

def main(args=None):
    print ("======HERE======")
    rclpy.init(args=args)
    global expid
    global runid
    global loc
    global topic
    global duration
    log_dir_path = os.path.join( "/home/minarady","wifi_ranger", "logs")
    minimal_subscriber = MinimalSubscriber(
        log_dir_path,
        expid,
        runid,
        loc,
        topic,
        duration)

    #rclpy.on_shutdown(shutdown_hook)
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
