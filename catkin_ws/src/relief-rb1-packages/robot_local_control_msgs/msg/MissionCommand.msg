# available commands
string GOTO_POSE=goto_pose
string GOTO_TAG=goto_tag
string GOTO_NODE=goto_node

# id set from the subsystem calling the service
string id
# command from the available commands
string command

MissionParamInt[] int_params
MissionParamFloat[] float_params
MissionParamString[] string_params
MissionParamBool[] bool_params



