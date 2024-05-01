/** ----------------------------GLOBAL VARIABLES---------------------------- **/
var following = false;



/**----------------------------------PARAM--------------------------------- **/



/**-----------------------------SUBSCRIBERS-------------------------------- **/
var isfollow_subscriber = new ROSLIB.Topic({
	ros : ros,
	name : namespace+'/pid_tf_follower/is_following',
	messageType : 'std_msgs/Bool'
});



/** ----------------------------SERVICE CLIENTS---------------------------- **/
// To select the person
var sc_select_person = new ROSLIB.Service({   
    ros : ros,
    name : namespace+'/init_people_followed',
    messageType : 'std_srvs/Empty'
});

// To (de)activate the follower
var sc_activate_follower = new ROSLIB.Service({   
    ros : ros,
    name : namespace+'/pid_tf_follower/enable',
    messageType : 'robotnik_msgs/enable_disable'
});
    


/** -------------------------------FUNCTIONS------------------------------- **/

function InitPersonDetector(){
    //window.alert("Error. map_server has to be switched off before running a mapping");
    if(confirm("Do you want to select the person followed?")){
        var init_req = new ROSLIB.ServiceRequest({			
        });
        sc_select_person.callService(init_req,function(res){
        });
    }
}

function startFollowing(){
    if(following){
        window.alert("Error. Follower is already running.");
        document.getElementById("monitor").value=document.getElementById("monitor").value+"\n Error. Follower node is already initiated.";
        document.getElementById("monitor").scrollTop = document.getElementById("monitor").scrollHeight;
    }else{
        console.log("starting1");
        var start_req = new ROSLIB.ServiceRequest({
            value: true				
        });
        console.log("starting2");
        sc_activate_follower.callService(start_req,function(res){
            console.log(res);
            if (res.ret){
                following = true;  
                document.getElementById("monitor").value=document.getElementById("monitor").value+"\n Follower node initiated.";
                document.getElementById("monitor").scrollTop = document.getElementById("monitor").scrollHeight;
            }
        });
    }
}

function stopFollowing(){

    if(!following){
        window.alert("Error. Follower is not running.");
        document.getElementById("monitor").value=document.getElementById("monitor").value+"\n Follower node stopped.";
        document.getElementById("monitor").scrollTop = document.getElementById("monitor").scrollHeight;
    }else{
        var stop_req = new ROSLIB.ServiceRequest({
            value: false				
        });
        sc_activate_follower.callService(stop_req,function(res){
            if (res.ret){
                following = false;
            }
        });

       
    }
}


/** ----------------------------TOPICS HANDLERS---------------------------- **/

isfollow_subscriber.subscribe(function(message){
    
    if(message.data){

        document.getElementById("following_status").style.color = "Green";       
        try{
            document.getElementById("button_start_following").disabled = true || disable_all_buttons;
            document.getElementById("button_stop_following").disabled = false || disable_all_buttons;
        }catch(e){
            
        }

    }else{

        document.getElementById("following_status").style.color = "Red";
        try{
            document.getElementById("button_start_following").disabled = false || disable_all_buttons;
            document.getElementById("button_stop_following").disabled = true || disable_all_buttons;
        }catch(e){
            
        }

    }

});




