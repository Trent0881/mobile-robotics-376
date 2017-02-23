// Path Action Client
// Written by Trent Ziemer 2/21/2017, heavily based on work from Dr. Wyatt Newman

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// #include <example_action_server/demoAction.h>
#include <pas/moveAction.h>
#include <std_msgs/Bool.h> 

bool g_obstacle_detected;
bool g_done_with_goal;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const pas::moveResultConstPtr& result) {
    ROS_INFO(" DoneCb: server responded with state [%s]", state.toString().c_str());
    g_done_with_goal = true;
    ROS_INFO("Result output from doneCb = %d", result->result);
}

void activeCb() {
    ROS_INFO("Active Callback");
}

void feedbackCb(const pas::moveFeedbackConstPtr& feedback) {
    if(feedback->is_spinning == true)
    {
        ROS_INFO("Server gives feedback that we are IN FACT SPINNING"); 
    }
    else
    {
        ROS_INFO("Server gives feedback that we are NOT SPINNING"); 
    }
}

void lidarAlarm(const std_msgs::Bool obstacleDetected)
{
    // If lidar alarm detects an obstacles, the path is no longer all clear
    if(obstacleDetected.data == true)
    {
        g_obstacle_detected = true;
    }
    else
    {
        g_obstacle_detected = false;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_client");


    
    ros::NodeHandle nh;

    // We want to listen for any notification of obstacles from the lidar alarm
    ros::Subscriber lidar_alarm = nh.subscribe("/lidar_alarm", 1, lidarAlarm); 

    pas::moveGoal goal;

    actionlib::SimpleActionClient<pas::moveAction> action_client("example_action", true);

    // attempt to connect to the server:
    ROS_INFO("Client is waiting for server");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("Client could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("Client has connected to action server");

    // Global feedback to change states
    g_done_with_goal = false;
    g_obstacle_detected = false;

    // Client states for the robot to be in (go forward or spin to avoid an obstacle)
    bool go = true;
    bool spin = false;

    while (true) {
        if(spin == false && go == true)
        {
            ROS_INFO("GOING");
            goal.x1 = 0.5; 
            goal.y1 = 1.5; 
            goal.theta1 = 3.14159/2; 

            goal.x2 = -0.5; 
            goal.y2 = 0; 
            goal.theta2 = -3.14159/2; 

            action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 


            // Wait until either done with goal or an obstacle has been detected to proceed
            while (g_done_with_goal == false && g_obstacle_detected == false)
            {
                ros::spinOnce();
            }
            if (g_obstacle_detected == true)
            {
                spin = true;
                go = false;
            }
        }
        else if(spin == true && go == false)
        {
                ROS_INFO("OBSTACLE, cancelling goals");
                action_client.cancelAllGoals(); 

                goal.x1 = 0; 
                goal.y1 = 0; 
                goal.theta1 = 3.14159/4; 

                goal.x2 = 0; 
                goal.y2 = 0; 
                goal.theta2 = 3.14159/4; 
                ROS_INFO("SENDING TURN GOAL");
                action_client.sendGoal(goal, &doneCb);

                // Regardless of obstacle detection, wait until done rotating
                while (g_done_with_goal == false)
                {
                    ros::spinOnce();
                }
                spin = false;
                go = true;
        }
        else
        {
            ROS_INFO("INVALID STATE, CHECK LOGIC");
        }
        // bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
    }

    return 0;
}

