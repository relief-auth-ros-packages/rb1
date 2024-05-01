# MISSION STATES
# States inside a mission
string MISSION_STATE_UNKNOWN=unknown
string MISSION_STATE_RUNNING=running
string MISSION_STATE_PAUSED=paused
string MISSION_STATE_FINISHED=finished
string MISSION_STATE_CANCELLED=cancelled
string MISSION_STATE_ERROR=error
string MISSION_STATE_QUEUED=queued
# running: mission is running
# paused: mission is paused
# finished: mission finished
# canceled: mission has been canceled
# error: an error occurred during the execution of the mission
# queued: the mission is queued


#
int32 id

# ID of the mision based on string format: GOTO [4,5,6]
string description

# running, paused, finished,...
string state


