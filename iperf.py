import logging
import os
import io
import time
import subprocess
import sys
from subprocess import check_output
import pdb


print (sys.argv)
expID = sys.argv[1]
runID = sys.argv[2]
params = sys.argv[3] 
all_params = params.split()
all_params.append("-T")
all_params.append("{}_{}".format(expID,runID))

print (all_params)
log_dir_path = os.path.join(os.path.dirname(__file__), "logs")
log_file_name = "{}_{}.json".format(expID,runID)
file_path = os.path.join(log_dir_path, log_file_name)
success = 0
tries = 1

while (success ==0 and tries >0):
    try:
        #out = subprocess.run( all_params, shell=True, check=True, capture_output=True)
        cmd= ' '.join(all_params)
        out= os.popen(cmd).read()
        clean_out = out.replace('\r','').replace('\n','')
        # subprocess.run( all_params, shell=True, check=True, capture_output=True)
        print ("logging")
        with open(file_path, 'w') as f:
            f.write('{}\n'.format(clean_out))
        success = 1
        # out.kill()  
    except subprocess.CalledProcessError as e:
        tries = tries - 1
        print ("not logging")
        print('return code =', e.returncode)
        print(e.output)
sys.exit()
