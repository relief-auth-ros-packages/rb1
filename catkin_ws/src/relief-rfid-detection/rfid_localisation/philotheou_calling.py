import time
import numpy as np
from main import *
from functions_for_app import *
import pandas

#filename="E:\\Experiments Library 2021\\EE_LIB_EXPS_2020_02_05\\rb1\\single_reader\\001625127C5D\\"
filename="C:\\Users\\Tasos Tzitzis\\Desktop\\6\\"
rfid_measurements_filelist=[]
rfid_measurements_filelist.append(filename+"results_001625127C5F.txt") 
rfid_measurements_filelist.append(filename+"results_001625127C5D.txt") 

rfid_antennas_poses_filelist=[]
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_1.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_2.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_3.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_4.txt") 

rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_1.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_2.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_3.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_4.txt")


robot_velocity=filename+"velocities.txt"

rfid_locations=filename+"rfid_results_by_python.txt"
rfid_locations_for_app=filename+"rfid_results_for_app.txt"

tags_infos=[]

storage_file=filename+"blabla.txt"

execution_timestamp=-1

existed_tags=[]
Results=pandas.DataFrame([])




start_time = time.time()
rfid_localization_output = rfid_localization(rfid_locations,tags_infos,robot_velocity,rfid_antennas_poses_filelist,rfid_measurements_filelist,existed_tags,execution_timestamp,Results)
elapsed_time = time.time() - start_time

existed_tags = rfid_localization_output[0]
execution_timestamp = rfid_localization_output[1]
Results = rfid_localization_output[2]


fig = plt.figure(1)
ax = fig.add_subplot(projection='3d')
for tag in existed_tags:
    ax.scatter(tag.x,tag.y,tag.z)
plt.xlim(1000,1400)
plt.ylim(500,700)

plt.show()




print("----------------------------------END ITERATION--------------------------------------------")

