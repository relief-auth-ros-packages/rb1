

from math import isnan
import numpy as np
import copy




from functions_parser import *
from functions_estimate_location import *
from classes import *
from functions_app import *


def handle_existed_tags(existed_tags,new_tags,Results,rfid_locations,storage_coords):


    existed_tags_EPCs = [x.EPC for x in existed_tags]

    for new_tag in new_tags: 
        
        
        if new_tag.EPC in existed_tags_EPCs:

            
            new_tag_index=existed_tags_EPCs.index(new_tag.EPC)
            tag=existed_tags[new_tag_index]

            concatenate_old_new_tag_measurements(new_tag,tag)
            tag.set_to_locate()

            if tag.to_locate>=1:
                tag = estimate_location(tag)
                tag = estimate_location_for_app(tag,storage_coords)
            #specific_existed_tag.print()

        else:

            tag=unwrap_for_new_tag(new_tag)
            tag.set_to_locate()
            existed_tags.append(tag)

            if existed_tags[-1].to_locate>=1:
                existed_tags[-1] = estimate_location(existed_tags[-1])
                existed_tags[-1] = estimate_location_for_app(existed_tags[-1],storage_coords)
            #specific_new_tag.print()

            tag = existed_tags[-1]
        
        #tag.print()
        start_time = time.time()
        if len(Results):
            Results.drop(Results[Results[0] ==tag.EPC].index, inplace = True)
        Estimation=np.hstack((tag.EPC,tag.x,tag.y,tag.z,tag.CI))
        Results=Results.append((pandas.DataFrame(Estimation)).T,ignore_index='True')
        Results.to_csv(rfid_locations, header=False, index=False)
        elapsed_time = time.time() - start_time
        

        #print("------------------------------------")
        #print(Results)
        #print("writing time: "+ str(elapsed_time))
        #print("------------------------------------")


    return existed_tags, Results


# %%

def parsing_data(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,execution_timestamp):

    readers= read_reader_readings(rfid_measurements_filelist,execution_timestamp)

    # ama o pinakas readers den periexei kanena object tote simainei oti den eixa txt gai rfid
    if len(readers)==0:
        print("No readers found--No rfid_measurements_filelist found")
        #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    readers=create_antenna_objects_for_each_reader(readers,rfid_antennas_poses_filelist,execution_timestamp)
    if len(readers)==0:
        print('No antenna poses')
        #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    Velocities=read_velocities(robot_velocity,execution_timestamp)
    if len(Velocities)==0:
        print('No velocities')
        #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    first_time_pose, last_time_pose=find_first_last_pose(readers[0].antennas[0].poses)
   

    Velocities = keep_robot_velocities_within_existed_poses(Velocities,first_time_pose,last_time_pose)
    if len(Velocities)==0:
        print('Velocities and Poses do not overlap')
        # return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    readers=keep_readers_with_overlapped_poses(readers,first_time_pose,last_time_pose)

            
    

    if len(readers)==0:
        print('No Reader measurements overlap with poses')
            #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp 
        return [],[],execution_timestamp

    velocity_data = find_nonmoving_periods(Velocities,readers[0].antennas[0].poses)
    
    unique_measured_tags_epcs=get_unique_measured_tag_epcs(readers)
    readers=create_poses_measurements(readers,velocity_data)
    
    execution_timestamp=last_time_pose
    
    return readers, unique_measured_tags_epcs,execution_timestamp




# %%
def initialise_measured_tags(readers,unique_measured_tags_epcs):  

    measured_tags=[]
    for epc in unique_measured_tags_epcs:
        
        tag_temp=Tag(epc)
        for r in readers:  #den einai adeio
            reader_temp=Reader(r.IP)

            for a in r.antennas:  #den einai adeio
                antenna_temp=Antenna(a.index)
                antenna_temp.measurements=a.measurements[np.where(a.epcs==epc)[0],:] #replaceeee
                if len(antenna_temp.measurements)!=0:
                    reader_temp.antennas.append(antenna_temp)
            
            if len(reader_temp.antennas)!=0:
                tag_temp.readers.append(reader_temp)
        
        measured_tags.append(tag_temp)

    return measured_tags



# %%
def preprocessing_and_localizing(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,rfid_locations,storage_file,existed_tags,execution_timestamp,Results):

    readers, unique_measured_tags_epcs, execution_timestamp = parsing_data(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,execution_timestamp)

    #print(execution_timestamp)

    if len(readers):

        #discriminate data according to each tag
        new_tags = initialise_measured_tags(readers,unique_measured_tags_epcs)
        
        storage_coords = get_storage_coordinates(storage_file)
        
        existed_tags, Results = handle_existed_tags(existed_tags,new_tags,Results,rfid_locations,storage_coords)

        print("new tags: " + str(len(new_tags)))
        print("existed_tags: " + str(len(existed_tags)))
  

    return existed_tags, execution_timestamp, Results




