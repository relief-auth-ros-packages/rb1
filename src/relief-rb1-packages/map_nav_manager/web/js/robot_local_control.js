/** ----------------------------GLOBAL VARIABLES---------------------------- **/
var picking = false;
var placing = false;

// Same variable is used in both, pick and place request.
// Now it doesn't use pick_frame_id, this can change in future.
var pick_req = new ROSLIB.ServiceRequest({
	procedure : {pick_frame_id: ''}
});

// Same variable is used in both, pick and place state request.
// id: -1 gets the state of the last procedure run. This can change in future.
var state_req = new ROSLIB.ServiceRequest({
	header : {
		id: -1,
		priority: 0,
		stamp: {
			secs: 0,
			nsecs: 0
		},
		name: ''
	}				
});

/**----------------------------------PARAM--------------------------------- **/
var use_rlc_param = new ROSLIB.Param({
	ros: ros,
	name: namespace + '/poi_interactive_marker/use_rlc',
});


/**-----------------------------SUBSCRIBERS-------------------------------- **/
var rlc_subscriber = new ROSLIB.Topic({
	ros : ros,
	name : namespace+'/robot_local_control/state',
	messageType : 'robot_local_control_msgs/Status'
});



/** ----------------------------SERVICE CLIENTS---------------------------- **/
// Clients to add procedures
var sc_pick_add = new ROSLIB.Service({  
	ros : ros,
	name : namespace+'/robot_local_control/NavigationComponent/PickComponent/add',
	messageType : 'robot_local_control_msgs/PickPetition'
});

var sc_place_add = new ROSLIB.Service({  
	ros : ros,
	name : namespace+'/robot_local_control/NavigationComponent/PlaceComponent/add',
	messageType : 'robot_local_control_msgs/PlacePetition'
});

// Clients to get the state
var sc_pick_state = new ROSLIB.Service({  
	ros : ros,
	name : namespace+'/robot_local_control/NavigationComponent/PickComponent/query_state',
	messageType : 'procedures_msgs/ProcedureQuery'
});

var sc_place_state = new ROSLIB.Service({  
	ros : ros,
	name : namespace+'/robot_local_control/NavigationComponent/PlaceComponent/query_state',
	messageType : 'procedures_msgs/ProcedureQuery'
});

var sc_control_mode = new ROSLIB.Service({
	ros : ros,
	name : namespace+'/robot_local_control/set_control_mode',
	messageType : 'robot_local_control_msgs/SetControlState'
});

// Client to set standby mode 
var sc_standby_mode = new ROSLIB.Service({
	ros : ros,
	name : namespace+'/safety_module/set_to_standby',
	messageType : 'std_srvs/SetBool'
});

/** -------------------------------FUNCTIONS------------------------------- **/
setInterval(stateRequest,2000);

function stateRequest(){
	if(picking){
		pickStateRequest();
	}

	if(placing){
		placeStateRequest();
	}
}

function pickStateRequest(){
	var current_state = "queued"
	var last_event = "added"
	var last_msg = ""
	sc_pick_state.callService(state_req, function(res){
		current_state = res.state.current_state;
		last_event = res.state.last_event;
		last_msg = res.last_message.data;
		if(current_state == "finished"){
			picking = false
		if(last_event == "abort"){
			alert(last_msg);
		}
	}
	});
}

function placeStateRequest(){
	var current_state = "queued"
	var last_event = "added"
	var last_msg = ""
	sc_place_state.callService(state_req, function(res){
		current_state = res.state.current_state;
		last_event = res.state.last_event;
		last_msg = res.last_message.data;
		if(current_state == "finished"){
			placing = false
		if(last_event == "abort"){
			alert(last_msg);
		}
	}
	});
}

function confirmPickProcedure(){
    if(confirm("Do you want to start picking?")){
		try {

			sc_pick_add.callService(pick_req, function(res){
            });
			picking = true;	

			//document.getElementById("button_pick_procedure").disabled = true;
        	//document.getElementById("button_place_procedure").disabled = false;
		}
		catch(err) {
    		console.log(err.message);
		}
    }
}

function confirmPlaceProcedure(){
    if(confirm("Do you want to start placing?")){
		try {
			sc_place_add.callService(pick_req, function(res){

            });	
			
			//document.getElementById("button_pick_procedure").disabled = true;
        	//document.getElementById("button_place_procedure").disabled = false;
		}
		catch(err) {
    		console.log(err.message);
		}
	}
}

function changeControlMode(buttonID){
	
	if(buttonID.startsWith('manual')){
		var control_mode_req = new ROSLIB.ServiceRequest({
			command: 'manual',			
		});
		sc_control_mode.callService(control_mode_req, function(res){
			if(res.success){
				document.getElementById(buttonID).classList.add('active');
				document.getElementById('autoButton').classList.remove('active');
				document.getElementById('followButton').classList.remove('active');
			}
		});
	}else if(buttonID.startsWith('auto')){
		var control_mode_req = new ROSLIB.ServiceRequest({
			command: 'auto',			
		});
		sc_control_mode.callService(control_mode_req, function(res){
			if(res.success){
				document.getElementById(buttonID).classList.add('active');
				document.getElementById('manualButton').classList.remove('active');
				document.getElementById('followButton').classList.remove('active');
			}
		});
	}else if(buttonID.startsWith('follow')){
		var control_mode_req = new ROSLIB.ServiceRequest({
			command: 'follow',			
		});
		sc_control_mode.callService(control_mode_req, function(res){
			console.log(res);
			if(res.success){
				document.getElementById(buttonID).classList.add('active');
				document.getElementById('manualButton').classList.remove('active');
				document.getElementById('autoButton').classList.remove('active');
			}
		});
	}
}

function startStandby(){
	var standby_req = new ROSLIB.ServiceRequest({
		data: true				
	});
	sc_standby_mode.callService(standby_req,function(res){
	});
}

function cancelStandby(){
	var standby_req = new ROSLIB.ServiceRequest({
		data: false				
	});
	sc_standby_mode.callService(standby_req,function(res){
	});
}

/** ----------------------------TOPICS HANDLERS---------------------------- **/

rlc_subscriber.subscribe(function(message){
	var idButton = message.control_state + "Button";
	//document.getElementById(idButton).classList.add('active');
	// After we get the first message this subscription is no longer needed.
	rlc_subscriber.unsubscribe();
});

use_rlc_param.get(function(param){
	if(param){
		var style = "display: true";
		document.getElementById('robot_local_control').setAttribute("style", style);
	}
});



