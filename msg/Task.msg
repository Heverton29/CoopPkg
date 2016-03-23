string name
uint8 id
geometry_msgs/Pose2D pose
time start
time end
time deadline
bool complete
uint8 state # NOT_ASSIGNED, //0
	    # WAITING,      //1
	    # EXECUTING,    //2
	    # ABORTED,      //3
	    # FAILED,       //4
	    # SUCCEEDED     //5
coop_pkg/Skill[] skills
