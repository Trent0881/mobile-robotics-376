// Path Action Server
// Written by Trent Ziemer 2/21/2017, heavily based on work from Dr. Wyatt Newman

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pas/moveAction.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

int g_count = 0;
bool g_count_failure = false;
geometry_msgs::Twist g_twist_cmd;

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
	actionlib::SimpleActionServer<pas::moveAction> as_;
    
    // here are some message types to communicate with our client(s)
    pas::moveGoal goal_; // goal message, received from client
    pas::moveResult result_; // put results here, to be sent back to the client when done w/ goal
    pas::moveFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    int countdown_val_;

public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<pas::moveAction>::GoalConstPtr& goal);
    void do_move(double distance);
    void do_spin(double spin_ang);
};

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "example_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("Starting my action server (constructor)");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

const double g_move_speed = 0.2; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.2; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;

ros::Publisher g_twist_commander; //global publisher object

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}



void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<pas::moveAction>::GoalConstPtr& goal) {
    ROS_INFO("Action server is executing a callback");

    float x_goal = goal->x1;
    float y_goal = goal->y1;
    float theta  = goal->theta1;

    ROS_INFO("First pose goal: X = %f, Y = %f, Theta = %f", x_goal, y_goal, theta);


    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(pow( pow(x_goal, 2) + pow(y_goal, 2), 0.5))/g_move_speed;
    g_twist_cmd.angular.z = 0.0;
    g_twist_cmd.linear.x = sgn(pow( pow(x_goal, 2) + pow(y_goal, 2), 0.5))*g_move_speed;
    while(timer<final_time) {
		// each iteration, check if cancellation has been ordered
		if (as_.isPreemptRequested()){	
			ROS_WARN("Goal was pre-empted, cancelling!");
			result_.result = 44;
			as_.setAborted(result_);
			return; 
		}
		ROS_INFO("Server providing feedback to client now");
		
		ROS_INFO("MOVING timer: %f", timer);
		g_twist_commander.publish(g_twist_cmd);
		timer+=g_sample_dt;
		feedback_.is_spinning = false;
		as_.publishFeedback(feedback_);
        loop_timer.sleep(); 
    }  

    // Make sure to stop afterwards
    do_halt();

    timer=0.0;
    final_time = fabs(theta)/g_spin_speed;
    g_twist_cmd.linear.x = 0.0;
    g_twist_cmd.angular.z= sgn(theta)*g_spin_speed;
    while(timer<final_time) {
		// each iteration, check if cancellation has been ordered
		if (as_.isPreemptRequested()){	
			ROS_WARN("Goal was pre-empted, cancelling!");
			result_.result = 55;
			as_.setAborted(result_);
			return; 
		}
		ROS_INFO("Server providing feedback to client now");
		
		ROS_INFO("SPINNING timer: %f", timer);
		g_twist_commander.publish(g_twist_cmd);
		timer+=g_sample_dt;
		feedback_.is_spinning = false;
		as_.publishFeedback(feedback_);
        loop_timer.sleep(); 
    }  

    // Make sure to stop afterwards
    do_halt(); 


    /// SECOND POSE

    x_goal = goal->x2;
    y_goal = goal->y2;
    theta  = goal->theta2;

    ROS_INFO("Second pose goal: X = %f, Y = %f, Theta = %f", x_goal, y_goal, theta);

	timer=0.0;
    final_time = fabs(pow( pow(x_goal, 2) + pow(y_goal, 2), 0.5))/g_move_speed;
    g_twist_cmd.angular.z = 0.0;
    g_twist_cmd.linear.x = sgn(pow( pow(x_goal, 2) + pow(y_goal, 2), 0.5))*g_move_speed;
    while(timer<final_time) {
		// each iteration, check if cancellation has been ordered
		if (as_.isPreemptRequested()){	
			ROS_WARN("Goal was pre-empted, cancelling!");
			result_.result = 44;
			as_.setAborted(result_);
			return; 
		}
		ROS_INFO("Server providing feedback to client now");
		
		ROS_INFO("MOVING timer: %f", timer);
		g_twist_commander.publish(g_twist_cmd);
		timer+=g_sample_dt;
		feedback_.is_spinning = false;
		as_.publishFeedback(feedback_);
        loop_timer.sleep(); 
    }  

    // Make sure to stop afterwards
    do_halt();

    timer=0.0;
    final_time = fabs(theta)/g_spin_speed;
    g_twist_cmd.linear.x = 0.0;
    g_twist_cmd.angular.z= sgn(theta)*g_spin_speed;
    while(timer<final_time) {
		// each iteration, check if cancellation has been ordered
		if (as_.isPreemptRequested()){	
			ROS_WARN("Goal was pre-empted, cancelling!");
			result_.result = 55;
			as_.setAborted(result_);
			return; 
		}
		ROS_INFO("Server providing feedback to client now");
		
		ROS_INFO("SPINNING timer: %f", timer);
		g_twist_commander.publish(g_twist_cmd);
		timer+=g_sample_dt;
		feedback_.is_spinning = false;
		as_.publishFeedback(feedback_);
        loop_timer.sleep(); 
    }  

    // Make sure to stop afterwards
    do_halt(); 

	result_.result = 0;
	    
    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_server"); // name this node 
  	ros::NodeHandle nh;
    
    ROS_INFO("Starting my action server");

    // Publish twist commands for the robot to consume
    g_twist_commander = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);  

    ExampleActionServer as_object;
    
    ROS_INFO("Spinning my action server");
 
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    
    while (!g_count_failure) {
        ros::spinOnce(); 
    }

    return 0;
}

