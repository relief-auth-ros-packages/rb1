#!/usr/bin/env python

import os
import pandas as pandas
import numpy as np
import math
from math import pi
import PhaseReLock
import PhaseUnwrapping





def interpolate(antenna_poses,rfid_readings,velocity_time,velocity_values):


        X_Antenna=antenna_poses[:,1]
        Y_Antenna=antenna_poses[:,2]
        Z_Antenna=antenna_poses[:,3]
        Theta_Antenna=antenna_poses[:,4]
        Time_ROS=antenna_poses[:,0]


        Tag_Time=rfid_readings[:,0]

        X_Tag=np.interp(Tag_Time,Time_ROS,X_Antenna)
        Y_Tag=np.interp(Tag_Time,Time_ROS,Y_Antenna)
        Theta_Tag=np.interp(Tag_Time,Time_ROS,Theta_Antenna)
        V_Tag=np.interp(Tag_Time,velocity_time,velocity_values)
        Z_Tag=Z_Antenna[0]*np.ones(len(X_Tag))

        X_Tag=X_Tag.reshape((len(X_Tag),1))
        Y_Tag=Y_Tag.reshape((len(X_Tag),1))
        Z_Tag=Z_Tag.reshape((len(X_Tag),1))
        V_Tag=V_Tag.reshape((len(X_Tag),1))
        Theta_Tag=Theta_Tag.reshape((len(X_Tag),1))

        interpolated_rfid_readings=np.hstack((rfid_readings, X_Tag, Y_Tag, Z_Tag, Theta_Tag, V_Tag))

        return interpolated_rfid_readings



def read_rfid_measurements(rfid_measurements_filelist,readers,antennas,execution_timestamp):

    flag_measurements=0
    read_text=[]

    rfid_measurements=[np.empty((0,5)) for _ in range(len(readers))]
    rfid_epcs=[np.empty((0,1)) for _ in range(len(readers))]

    for i in range(0,len(rfid_measurements_filelist)):


        filename=rfid_measurements_filelist[i]

        search_for_this="results_"
        start_index=filename.find(search_for_this)
        reader_index=start_index+len(search_for_this)

        cell_index=np.where(np.array(readers)==filename[reader_index:reader_index+12])[0][0]


        if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):

                read_text=pandas.read_csv(filename, sep=",", header=None)

                read_text=read_text.drop(read_text[read_text[0]<execution_timestamp].index)


                if len(read_text):
                    rfid_measurements[cell_index]=read_text.select_dtypes(include=['float64','int64'])
                    rfid_measurements[cell_index]=rfid_measurements[cell_index].to_numpy()
                    rfid_epcs[cell_index]=read_text.select_dtypes(include=['object'])
                    rfid_epcs[cell_index]=rfid_epcs[cell_index].to_numpy()
                    flag_measurements=1

                else:
                    print('No measurements for Reader '+filename[reader_index:reader_index+12])

        else:

            print('No txt for Reader '+filename[reader_index:reader_index+12])

        for i in range(0,len(rfid_epcs)):
            if len(rfid_epcs[i]):
                for j in range(0,len(rfid_epcs[i])):
                    rfid_epcs[i][j][0]=rfid_epcs[i][j][0].replace(" ","")

    return rfid_measurements, rfid_epcs, flag_measurements








def create_readers_antennas_tables(rfid_antennas_poses_filelist):


    readers=[]
    antennas=[]

    for i in range(0,len(rfid_antennas_poses_filelist)):

        filename=rfid_antennas_poses_filelist[i]
        search_for_this="pose_reader_"
        start_index=filename.find(search_for_this)
        reader_index=start_index+len(search_for_this)

        if filename[reader_index:reader_index+12] not in readers:
            readers.append(filename[reader_index:reader_index+12])

    global_start_index=""
    antennas=[[] for _ in range(len(readers))]
    for i in range(0,len(readers)):

        for j in range(0,len(rfid_antennas_poses_filelist)):

            filename=rfid_antennas_poses_filelist[j]
            search_for_this="pose_reader_"+readers[i]+"_antenna_"
            start_index=filename.find(search_for_this)
            if start_index!=-1:
                global_start_index=start_index
                antenna_index=start_index+len(search_for_this)

                if filename[antenna_index] not in antennas[i]:

                    antennas[i].append(filename[antenna_index])



#    print(readers)
#    print(antennas)
    return readers, antennas, global_start_index




def read_velocities(robot_velocity,execution_timestamp):

    filename=robot_velocity
    if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):

        Velocities = pandas.read_csv(filename, sep=",", header=None)

        Velocities=Velocities.drop(Velocities[Velocities[0]<execution_timestamp].index)

        Velocities=Velocities.to_numpy()

        return Velocities

    else:
        print('No txt for Robot velocities')
        return []



def read_antenna_poses(rfid_antennas_poses_filelist,readers,antennas,global_start_index,execution_timestamp):


    reader_antenna_poses=[[] for _ in range(len(readers))]
    for i in range(0,len(readers)):
            reader_antenna_poses[i]=[np.empty((0,5)) for _ in range(len(antennas[i]))]

    flag_poses=0

    for i in range(0,len(readers)):

        for j in range(0,len(antennas[i])):

            filename=rfid_antennas_poses_filelist[0][0:global_start_index]+"pose_reader_"+readers[i]+"_antenna_"+antennas[i][j]+".txt"
#            print(filename)
            if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):

                read_txt=pandas.read_csv(filename, sep=",", header=None)

                read_txt=read_txt.drop(read_txt[read_txt[0]<execution_timestamp].index)
#                print(len(read_txt))

                if len(read_txt):
                    reader_antenna_poses[i][int(antennas[i][j])-1]=read_txt.to_numpy()
                    flag_poses=1
                else:
                    print('No poses for Reader '+readers[i]+', antenna '+antennas[i][j])

            else:

                print('No txt for '+readers[i]+'s antenna '+antennas[i][j])

    return reader_antenna_poses, flag_poses


def read_antenna_polarities(readers_antennas_polarities,readers,antennas):


    reader_antenna_polarities=[[] for _ in range(len(readers))]
    for i in range(0,len(readers)):
            reader_antenna_polarities[i]=[np.empty((0,1)) for _ in range(len(antennas[i]))]

    filename=readers_antennas_polarities

    if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):

        flag_polarities=1

        read_txt=pandas.read_csv(filename, sep=",", header=None)
        antenna_info=read_txt.select_dtypes(include=['float64','int64'])
        antenna_info=antenna_info.to_numpy()
        reader_mac=read_txt.select_dtypes(include=['object'])
        reader_mac=reader_mac.to_numpy()

        for i in range(0,len(reader_mac)):


           reader_index=np.where(np.array(readers)==reader_mac[i])[0][0]
           antenna_index=int(antenna_info[i,0])-1

           reader_antenna_polarities[reader_index][antenna_index]=antenna_info[i,1]

    else:

          print('No polarities.txt')
          flag_polarities=0


    return reader_antenna_polarities, flag_polarities





def find_nonmoving_periods(Velocities,reader_antenna_poses):


    #find when the robot was static.
    # this will help later to reject such data

    # thewrw oti to robot itan akinito otna oles oi sinistwses velocity einai miden
    velocity_time=Velocities[:,0]
    velocity_values=np.sqrt(Velocities[:,1]**2+Velocities[:,2]**2+Velocities[:,3]**2+Velocities[:,4]**2+Velocities[:,5]**2+Velocities[:,6]**2)

    velocity_values[velocity_values>0]=1;

    #vriskw pote stamataei kai ksekinaei
    start_stop=np.diff(velocity_values)
    start_moving=np.where(start_stop==1)[0]+1
    stop_moving=np.where(start_stop==-1)[0]+1


    #prosthetw analoga tin prwti kai tin teleutaia timi
    if velocity_values[0]==0:
        stop_moving=np.insert(stop_moving,0,0)

    if velocity_values[-1]==0:
        start_moving=np.append(start_moving,len(velocity_values)-1)


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

            stop_pose=np.where(reader_antenna_poses[0][0][:,0]<=non_moving_periods[0,i])[0]
            time_stop_poses[i]=reader_antenna_poses[0][0][stop_pose[-1],0]
            start_pose=np.where(reader_antenna_poses[0][0][:,0]>=non_moving_periods[1,i])[0]
            time_start_poses[i]=reader_antenna_poses[0][0][start_pose[0],0]



        non_moving_periods_poses=np.vstack((time_stop_poses,time_start_poses))


        for i in range(0,non_moving_periods_poses.shape[1]):

            velocity_values[(velocity_time>=non_moving_periods_poses[0,i]) & (velocity_time<=non_moving_periods_poses[1,i])]=0

    return velocity_values, velocity_time




def main(rfid_locations, \
    tags_infos, \
    robot_velocity, \
    readers_antennas_polarities, \
    rfid_antennas_poses_filelist,\
    rfid_measurements_filelist,\
    tag_measurements,\
    tag_unwrapped_measurements,\
    unique_tag,\
    execution_timestamp,\
    reader_antenna_lamda):

    if False:
        print(rfid_locations)
        print(tags_infos)
        print(robot_velocity)
        print(rfid_antennas_poses_filelist)
        print(rfid_measurements_filelist)




    readers, antennas, global_start_index =  create_readers_antennas_tables(rfid_antennas_poses_filelist)
    if len(readers)==0:
        print("No readers found")
        return [tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda]

    reader_antenna_poses, flag_poses = read_antenna_poses(rfid_antennas_poses_filelist,readers,antennas,global_start_index,execution_timestamp)
    if flag_poses==0:
        print('No Antenna poses at all')
        return [tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda]


    reader_antenna_polarities,flag_polarities=read_antenna_polarities(readers_antennas_polarities,readers,antennas)
    if flag_polarities==0:
        print('No polarities')
        return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda


    rfid_measurements,rfid_epcs, flag_measurements=read_rfid_measurements(rfid_measurements_filelist,readers,antennas,execution_timestamp)
    if flag_measurements==0:
        print('No RFID measurements at all')
        return [tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda]

    Velocities = read_velocities(robot_velocity,execution_timestamp)
    if len(Velocities)==0:
        print('No velocities')
        return [tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda]










    #keep velocities and tag readings for existing poses


    first_time_pose=reader_antenna_poses[0][0][0,0]
    last_time_pose=reader_antenna_poses[0][0][-1,0]



    Velocities=Velocities[(Velocities[:,0]>=first_time_pose) & (Velocities[:,0]<=last_time_pose)]
    if len(Velocities)==0:
        print('Velocities and Poses do not overlap')
        return [tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda]

    flag_overlap=0
    for i in range(0,len(rfid_measurements)):
        if len(rfid_measurements[i])!=0:
            rfid_epcs[i]=rfid_epcs[i][(rfid_measurements[i][:,0]>=first_time_pose) & (rfid_measurements[i][:,0]<=last_time_pose)]
            rfid_measurements[i]=rfid_measurements[i][(rfid_measurements[i][:,0]>=first_time_pose) & (rfid_measurements[i][:,0]<=last_time_pose)]

            if len(rfid_measurements[i]):
                flag_overlap=1
            else:
                print('Reader '+readers[i]+'readings poses do not overlap')

    if flag_overlap==0:
        print('No Reader measurements overlap with poses')
        return [tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda]




    velocity_values, velocity_time = find_nonmoving_periods(Velocities,reader_antenna_poses)



    #############  DATA FOR EACH ANTENNA ###########################
    if len(reader_antenna_lamda) == 0:
        reader_antenna_lamda=[[] for _ in range(len(readers))]

        for i in range(0,len(readers)):
            reader_antenna_lamda[i]=[np.empty((0,1)) for _ in range(len(antennas[i]))]


    reader_antenna_measurements=[[] for _ in range(len(readers))]
    reader_antenna_epcs=[[] for _ in range(len(readers))]
    reader_antenna_measurements_poses=[[] for _ in range(len(readers))]

    for i in range(0,len(readers)):

            reader_antenna_measurements[i]=[np.empty((0,4)) for _ in range(len(antennas[i]))]
            reader_antenna_epcs[i]=[np.empty((0,1)) for _ in range(len(antennas[i]))]
            reader_antenna_measurements_poses[i]=[np.empty((0,9)) for _ in range(len(antennas[i]))]


    for i in range(0,len(readers)):

        for j in range(0,len(antennas[i])):

            reader_antenna_measurements[i][int(antennas[i][j])-1]=rfid_measurements[i][rfid_measurements[i][:,1]==int(antennas[i][j]),0:-1]
            reader_antenna_epcs[i][int(antennas[i][j])-1]=rfid_epcs[i][rfid_measurements[i][:,1]==int(antennas[i][j]),:]
            reader_antenna_freq=rfid_measurements[i][rfid_measurements[i][:,1]==int(antennas[i][j]),-1]
            if len(reader_antenna_freq)!=0:
                reader_antenna_lamda[i][int(antennas[i][j])-1]=100000*299.792458/reader_antenna_freq[0]




     #############  INTERPOLATE POSES  ###########################


    for i in range(0,len(reader_antenna_measurements)):

        for j in range(0,len(reader_antenna_measurements[i])):

            if len(reader_antenna_measurements[i][j])!=0:
                reader_antenna_measurements_poses[i][j]=interpolate(reader_antenna_poses[i][j],reader_antenna_measurements[i][j],velocity_time,velocity_values)



    #############  READ PREVIOUS ESTIMATIONS  ###########################

    filename=rfid_locations
    if (os.path.exists(filename)) and ( not (os.stat(filename).st_size == 0)):

        Results = pandas.read_csv(filename, sep=",", header=None)
#        Results_Locations=Results.select_dtypes(include=['float64','int64']).to_numpy()
#        Results_EPCs=Results.select_dtypes(include=['object']).to_numpy()
        Results_EPCs=Results[0].to_numpy()
        Results_EPCs=[x.replace(" ","") for x in Results_EPCs]


    else:
        print('No Tag Locations estimated')
        Results_EPCs=[]
        Results=pandas.DataFrame([])





    unique_tag_new=[]

    for i in range(0,len(readers)):
        unique_tag_new=np.hstack((unique_tag_new,np.unique(rfid_epcs[i])))

    unique_tag_new=np.unique(unique_tag_new)

    unique_tag_new=[x.replace(" ","") for x in unique_tag_new]




    ###################### FOR EVERY TAG ##############################


    for tag_new in range(0,len(unique_tag_new)):

        Estimation_new=[]

        if unique_tag_new[tag_new] in unique_tag:

#            print('if')
            tag_new_index=unique_tag.index(unique_tag_new[tag_new])

            flag_new_unwrapped_data=0

            for i in range(0,len(reader_antenna_measurements_poses)):

                for j in range(0,len(reader_antenna_measurements_poses[i])):

                    if len(reader_antenna_measurements_poses[i][j])!=0:
                        new_data=reader_antenna_measurements_poses[i][j][np.where(reader_antenna_epcs[i][j]==unique_tag_new[tag_new])[0],:]
                        tag_measurements[tag_new_index][i][j]=np.vstack((tag_measurements[tag_new_index][i][j],new_data))
                        if len(new_data)!=0:
#                            print('new data found')
                            old_unwrapped_measurements=tag_unwrapped_measurements[tag_new_index][i][j]
                            tag_unwrapped_measurements[tag_new_index][i][j]=PhaseUnwrapping.PhaseUnwrapping(tag_measurements[tag_new_index][i][j],reader_antenna_lamda[i][j],reader_antenna_polarities[i][j])
                            if len(old_unwrapped_measurements)!=len(tag_unwrapped_measurements[tag_new_index][i][j]):
                                flag_new_unwrapped_data=1

        else:

#            print('else')
            tag_measurements_new=[[] for _ in range(len(readers))]
            tag_unwrapped_measurements_new=[[] for _ in range(len(readers))]

            flag_new_unwrapped_data=0

            for i in range(0,len(readers)):
                tag_measurements_new[i]=[np.empty((0, 9)) for _ in range(len(antennas[i]))]
                tag_unwrapped_measurements_new[i]=[np.empty((0, 6)) for _ in range(len(antennas[i]))]

                for j in range(0,len(reader_antenna_measurements_poses[i])):

                    if len(reader_antenna_measurements_poses[i][j])!=0:
                        tag_measurements_new[i][j]=reader_antenna_measurements_poses[i][j][np.where(reader_antenna_epcs[i][j]==unique_tag_new[tag_new])[0],:]
                        if len(tag_measurements_new[i][j])!=0:
#                            print('new data found')
                            tag_unwrapped_measurements_new[i][j]=PhaseUnwrapping.PhaseUnwrapping(tag_measurements_new[i][j],reader_antenna_lamda[i][j],reader_antenna_polarities[i][j])
                            if len(tag_unwrapped_measurements_new[i][j])!=0:
                                flag_new_unwrapped_data=1

            unique_tag.append(unique_tag_new[tag_new].replace(" ",""))
            tag_measurements.append(tag_measurements_new)
            tag_unwrapped_measurements.append(tag_unwrapped_measurements_new)


        if flag_new_unwrapped_data==1:
            tag_estimated_location=PhaseReLock.PhaseReLock(tag_unwrapped_measurements[tag_new],reader_antenna_lamda,reader_antenna_polarities)

            Estimation_new=np.hstack((unique_tag_new[tag_new].replace(" ",""),"Book",[0,0,0],tag_estimated_location))
            if unique_tag_new[tag_new].replace(" ","") in Results_EPCs:
                results_tag_index=Results_EPCs.index(unique_tag_new[tag_new].replace(" ",""))
                Results.loc[results_tag_index]=Estimation_new
            else:
    #            print('append')
                Results=Results.append((pandas.DataFrame(Estimation_new)).T,ignore_index='True')






    filename=rfid_locations

    if os.path.exists(filename):
        os.remove(filename)

    Results.to_csv(filename, header=False, index=False)



    execution_timestamp=last_time_pose


    return [tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp,reader_antenna_lamda]
