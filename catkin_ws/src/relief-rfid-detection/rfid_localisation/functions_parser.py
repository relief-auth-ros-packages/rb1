import os
import math
import pandas
import time
import numpy as np
import copy
from math import pi


from functions_phase_unwrapping import *
from classes import *




# %%
def read_reader_readings(rfid_measurements_filelist,execution_timestamp):    
    

    readers=[] # a list that contains objects of class Reader
    
    #diavazwnatas ta results blepw poioi readers dwsan metriseis
    for i in range(0,len(rfid_measurements_filelist)):
    
        filename=rfid_measurements_filelist[i]
        search_for_this="results_"
        start_index=filename.find(search_for_this)
        reader_index=start_index+len(search_for_this) #vriskei to IP poiwn reader iparxoun sta txt arxeia twn poses
        
        reader_temp=Reader(filename[reader_index:reader_index+12])
        if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):

            rfid_readings=pandas.read_csv(filename, sep=", ", header=None)
            rfid_readings=rfid_readings.drop(rfid_readings[rfid_readings[0]<execution_timestamp].index) 

            if len(rfid_readings):

                reader_temp.readings=(rfid_readings.select_dtypes(include=['float64','int64'])).to_numpy()                
                reader_temp.epcs=(rfid_readings.select_dtypes(include=['object'])).to_numpy()

                reader_temp.readings[:,2]=-reader_temp.readings[:,2]+2*pi
                reader_temp.readings[:,4]=100000*299.792458/reader_temp.readings[:,4]

                #reader_temp.epcs=[x.replace(" ","") for x in reader_temp.epcs]

                readers.append(reader_temp)
                
            else:
                print('No measurements for Reader '+reader_temp.IP)
        else:
            print('rfid_measurements.txt is empty or does not exist for Reader '+reader_temp.IP)    
        

            

    return readers



# %%
def read_antenna_poses(rfid_antennas_poses_filelist,IP,execution_timestamp):
    antennas=[]

    global_start_index=""

        
    for j in range(0,len(rfid_antennas_poses_filelist)):
        
        filename=rfid_antennas_poses_filelist[j]
        search_for_this="pose_reader_"+IP+"_antenna_"
        start_index=filename.find(search_for_this)

        if start_index!=-1:
            global_start_index=start_index
            antenna_index=start_index+len(search_for_this)  
            
            antenna_temp=Antenna(filename[antenna_index])
            if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):

                antenna_poses=pandas.read_csv(filename, sep=", ", header=None)
                antenna_poses=antenna_poses.drop(antenna_poses[antenna_poses[0]<execution_timestamp].index) 
                
                if len(antenna_poses):
                    antenna_temp.poses=antenna_poses.to_numpy()
                    antennas.append(antenna_temp)
                else:
                    print('No poses for Reader '+IP+', antenna '+antenna_temp.index)
            else:
                print('rfid_antennas_poses.txt is empty or does not exist for Reader '+IP+', antenna '+antenna_temp.index)
                                        
                   
    return antennas


# %%
def read_velocities(robot_velocity,execution_timestamp):
    
    filename=robot_velocity
    if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):
          
        Velocities = pandas.read_csv(filename, sep=", ", header=None)
        Velocities=Velocities.drop(Velocities[Velocities[0]<execution_timestamp].index) 
        
        Velocities=Velocities.to_numpy()
        
        return Velocities
    
    else:
        print('robot_velocity.txt is empty or does not exist')
        return []


# %%
def find_nonmoving_periods(Velocities,reader_antenna_poses):    
       
    
    #find when the robot was static.
    # this will help later to reject such data
    
    # thewrw oti to robot itan akinito otna oles oi sinistwses velocity einai miden
    velocity_time=Velocities[:,0]
    #velocity_values=np.sqrt(Velocities[:,1]**2+Velocities[:,2]**2+Velocities[:,3]**2+Velocities[:,4]**2+Velocities[:,5]**2+Velocities[:,6]**2)
    velocity_values=100*np.abs(Velocities[:,1])
    
    velocity_values_binary=copy.copy(velocity_values)
    velocity_values_binary[velocity_values_binary>0]=1
    
    #vriskw pote stamataei kai ksekinaei
    start_stop=np.diff(velocity_values_binary)
    start_moving=np.where(start_stop==1)[0]+1
    stop_moving=np.where(start_stop==-1)[0]+1
    
    
    #prosthetw analoga tin prwti kai tin teleutaia timi
    if velocity_values_binary[0]==0:
        stop_moving=np.insert(stop_moving,0,0)
    
    if velocity_values_binary[-1]==0:
        start_moving=np.append(start_moving,len(velocity_values_binary)-1)
    
    
    start_stop=np.vstack((stop_moving, start_moving))
    
    #twra exw tous xronous pou to robot itan akinito
    non_moving_periods=velocity_time[start_stop]
    
    #vriskw tis poses pou antistoixoun eksw apo to moving_periods
    if non_moving_periods.size>0:
    
        time_stop_poses=[None]*non_moving_periods.shape[1]
        time_start_poses=[None]*non_moving_periods.shape[1]
        
        # efoson ta poses exoun idious xronous mou arkei ena apo ayta na vrw
        #epeidi malista tha stelnei ola ta poses apo kathe keraia tha mpainei stin prwti if
        for i in range(0,non_moving_periods.shape[1]):

            stop_pose=np.where(reader_antenna_poses[:,0]<=non_moving_periods[0,i])[0]
            time_stop_poses[i]=reader_antenna_poses[stop_pose[-1],0]
            start_pose=np.where(reader_antenna_poses[:,0]>=non_moving_periods[1,i])[0]
            time_start_poses[i]=reader_antenna_poses[start_pose[0],0]

        
        non_moving_periods_poses=np.vstack((time_stop_poses,time_start_poses))
        
        
        for i in range(0,non_moving_periods_poses.shape[1]):
            
            velocity_values_binary[(velocity_time>=non_moving_periods_poses[0,i]) & (velocity_time<=non_moving_periods_poses[1,i])]=0

    velocity_data = np.vstack((velocity_time,velocity_values,velocity_values_binary))
    
    return velocity_data.T


# %%
def remove_empty_readers(readers):
    
    temp_readers=[r for r in readers if len(r.readings)]
    readers=temp_readers

    return readers


# %%
def create_antenna_objects_for_each_reader(readers,rfid_antennas_poses_filelist,execution_timestamp):

    for r in readers:
        r.antennas=read_antenna_poses(rfid_antennas_poses_filelist,r.IP,execution_timestamp)
        if len(r.antennas)==0:
            print('No antennas found---No rfid_antennas_poses_filelist found')
            return []
            #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        for a in r.antennas:
            if a.len_poses()==0:
                print('No poses for the robot')
                return []
                #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
    return readers


# %%
def keep_robot_velocities_within_existed_poses(Velocities,first_time_pose,last_time_pose):
    Velocities=Velocities[(Velocities[:,0]>=first_time_pose) & (Velocities[:,0]<=last_time_pose)]
    return Velocities


# %%
def find_first_last_pose(reader_antenna_poses):

    first_time_pose=reader_antenna_poses[0,0]
    last_time_pose=reader_antenna_poses[-1,0]

    return first_time_pose, last_time_pose


# %%
def keep_readers_with_overlapped_poses(readers,first_time_pose,last_time_pose):
    for r in readers:
        r=keep_specific_reader_readings_within_existed_poses(r,first_time_pose,last_time_pose)

    readers=remove_empty_readers(readers)

    return readers


# %%
def keep_specific_reader_readings_within_existed_poses(reader,first_time_pose,last_time_pose):

    reader.epcs=reader.epcs[(reader.readings[:,0]>=first_time_pose) & (reader.readings[:,0]<=last_time_pose)]
    reader.readings=reader.readings[(reader.readings[:,0]>=first_time_pose) & (reader.readings[:,0]<=last_time_pose)]

    if len(reader.readings)==0:
        print('Reader '+reader.IP+'readings poses do not overlap')

#girnaei  reader pou mporei na min exei readings
    return reader 


# %%
def interpolate_poses(antenna_poses,antenna_readings,velocity_data):
        
        
        X_ROS=100*antenna_poses[:,1]
        Y_ROS=100*antenna_poses[:,2]
        Z_ROS=100*antenna_poses[:,3]
        Theta_ROS=antenna_poses[:,4]
        Time_ROS=antenna_poses[:,0]
        
        
        Time_Reader=antenna_readings[:,0]
        
        X_Reader=np.interp(Time_Reader,Time_ROS,X_ROS)
        Y_Reader=np.interp(Time_Reader,Time_ROS,Y_ROS)
        Theta_Reader=np.interp(Time_Reader,Time_ROS,Theta_ROS)
        V_Reader=np.interp(Time_Reader,velocity_data[:,0],velocity_data[:,1])
        V_Reader_binary=np.interp(Time_Reader,velocity_data[:,0],velocity_data[:,2])
        Z_Reader=Z_ROS[0]*np.ones(len(X_Reader))
        
        X_Reader=X_Reader.reshape((len(X_Reader),1))
        Y_Reader=Y_Reader.reshape((len(X_Reader),1))
        Z_Reader=Z_Reader.reshape((len(X_Reader),1))
        V_Reader=V_Reader.reshape((len(X_Reader),1))
        V_Reader_binary=V_Reader_binary.reshape((len(X_Reader),1))
        Theta_Reader=Theta_Reader.reshape((len(X_Reader),1))
        
        interpolated_rfid_readings=np.hstack((antenna_readings, X_Reader, Y_Reader, Z_Reader, Theta_Reader, V_Reader, V_Reader_binary))

        return interpolated_rfid_readings


# %%
def find_poses_measurements_of_specific_antenna(reader_readings,reader_reading_epcs,antenna_index,antenna_poses,velocity_data):

    antenna_measurements=reader_readings[reader_readings[:,1]==antenna_index,:]
    epcs=reader_reading_epcs[reader_readings[:,1]==antenna_index,:]
    augmented_measurements=interpolate_poses(antenna_poses,antenna_measurements,velocity_data)
           
    return augmented_measurements,epcs


# %%
def create_poses_measurements(readers,velocity_data):

        for r in readers:#[x for x in readers if len(x.readings)!=0]: 
        
                for a in r.antennas:
                
                        a.measurements, a.epcs = find_poses_measurements_of_specific_antenna(r.readings,r.epcs,int(a.index),a.poses,velocity_data)
  

                #r.readings=[]

        readers=clear_empty_readers(readers)

        return readers


# %%
def clear_empty_readers(readers):
    for r in readers:
        temp_antennas = [a for a in r.antennas if len(a.measurements)]
        r.antennas=temp_antennas
    return readers


    
# %%
def get_unique_measured_tag_epcs(readers):
    unique_tags_epcs=[]

    for r in readers:#[x for x in readers if len(x.readings)!=0]: 
        unique_tags_epcs=np.hstack((unique_tags_epcs,np.unique(r.epcs)))

    unique_measured_tags_epcs=np.unique(unique_tags_epcs)
    
    #unique_measured_tags_epcs=[x.replace(" ","") for x in unique_measured_tags_epcs]
    
    return unique_measured_tags_epcs









# %%
def concatenate_old_new_tag_measurements(new_tag,old_tag):
    
    #pairnw apo to measured_tags kathe pinaka metrisewn, ton anazitw sto estimated tag kai to enwnw
    for r_new in new_tag.readers: 
        #r.print()
        IP=r_new.IP
        r_old=old_tag.find_reader_with_IP(IP)
         #reader=[]
        if r_old:
            for a_new in r_new.antennas:
                #print(a_new.index)
                #print('a_new_measurements befroe stack')
                #print(len(a_new.measurements))
                index=a_new.index
                a_old=r_old.find_antenna_with_index(index)
                #an ipirxan apo prin metriseis 
                if a_old:
                    #print('a_old_measurements before stack')
                    #print(len(a_old.measurements))

                    a_old.measurements = np.vstack((a_old.measurements,a_new.measurements))
                    temp_unwrapped_measurements = get_unwrapped_measurements(copy.copy(a_old.measurements))
                    if len(temp_unwrapped_measurements)>0:    
                        #ekane unwrap kai einai diaforetiko apo prin  
                        if not (np.array_equal(temp_unwrapped_measurements,a_old.unwrapped_measurements)):
                            a_old.to_locate=1
                        #ekane unwrap kai einai idio me prin
                        else:
                            a_old.to_locate=0
                        a_old.unwrapped_measurements = temp_unwrapped_measurements
                    else:
                        #den ekane unwrap alla exei me wrapped kainouria
                        a_old.to_locate=2


                #an den ipirxan metriseis apo prin
                else:
                    temp_unwrapped_measurements = get_unwrapped_measurements(copy.copy(a_new.measurements))
                    if len(temp_unwrapped_measurements)>0:     
                        #ekane unwrap kai einai diaforetiko apo prin (panta vsk giati einai neea keraia)
                        if not (np.array_equal(temp_unwrapped_measurements,a_new.unwrapped_measurements)):
                            a_new.to_locate=1
                        else:
                            a_new.to_locate=0
                        a_new.unwrapped_measurements = temp_unwrapped_measurements
                    else:
                        a_new.to_locate=2
                    r_old.antennas.append(a_new)

        #print(len(tag.find_reader_with_IP(IP).find_antenna_with_index(index).measurements))

        #keraia apo neo reader
        else:  #antenna 
            for a_new in r_new.antennas:
                temp_unwrapped_measurements = get_unwrapped_measurements(copy.copy(a_new.measurements))
                #ekane unwrap
                if len(temp_unwrapped_measurements)>0:    
                    #einai diaforetiko apo print (panta vsk)   
                    if not (np.array_equal(temp_unwrapped_measurements,a_new.unwrapped_measurements)):
                        a_new.to_locate=1
                    else: 
                        a_new.to_locate=0
                    a_new.unwrapped_measurements = temp_unwrapped_measurements
                #den ekane unwrap
                else:
                    a_new.to_locate=2
                    
            old_tag.readers.append(r_new)
    


# %%
def unwrap_for_new_tag(tag):
    #tag.print()
    #print("unwrap for new tag")
    for r in tag.readers:
        for a in r.antennas:
            temp_unwrapped_measurements = get_unwrapped_measurements(copy.copy(a.measurements))  
            #ekane unwrap
            if len(temp_unwrapped_measurements)>0: 
                if not (np.array_equal(temp_unwrapped_measurements,a.unwrapped_measurements)):
                    a.to_locate=1
                else:
                    a.to_locate=0
                a.unwrapped_measurements = temp_unwrapped_measurements
            else:
                a.to_locate=2
    
    return tag


