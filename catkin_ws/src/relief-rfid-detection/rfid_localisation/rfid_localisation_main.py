import numpy as np
import os
import pandas
import matplotlib.pyplot as plt

from functions_main import *





def rfid_localization(rfid_locations,tags_infos,storage_file,robot_velocity,rfid_antennas_poses_filelist,rfid_measurements_filelist,existed_tags,execution_timestamp,Results):


    #parsing the data and put them in existed_tags
    start_time = time.time()

    try:
      existed_tags, execution_timestamp, Results = preprocessing_and_localizing(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,rfid_locations,storage_file,existed_tags,execution_timestamp,Results)
    except:
      print("paixtike mlkia filos")

    elapsed_time = time.time() - start_time
    #print("time for unwrap: "+ str(elapsed_time))
    print("execution time: "+ str(elapsed_time))



    return [existed_tags, execution_timestamp, Results]




