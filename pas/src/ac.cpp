// Path Action Client
// Written by Trent Ziemer 2/21/2017, heavily based on work from Dr. Wyatt Newman

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// #include <example_action_server/demoAction.h>
#include <pas/moveAction.h>
#include <std_msgs/Bool.h> 

bool g_allClear;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const pas::moveResultConstPtr& result) {
    ROS_INFO(" DoneCb: server responded with state [%s]", state.toString().c_str());

    ROS_INFO("Result output from doneCb = %d", result->result);
}

void activeCb() {
    ROS_INFO(" Active Callback: server responded with stat");

    ROS_INFO("Result output from activeCb =");}

void feedbackCb(const pas::moveFeedbackConstPtr& feedback) {
    ROS_INFO(" Feedback Callback: server responded with ...");

    ROS_INFO("Result output from feedbackCb = %d", feedback->is_spinning);
}

void lidarAlarm(const std_msgs::Bool obstacleDetected)
{
    // If lidar alarm detects an obstacles, the path is no longer all clear
    if(obstacleDetected.data == true)
    {
        g_allClear = false;
    }
    else
    {
        g_allClear = true;
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

    while (true) {
        goal.x1 = 0.5; 
        goal.y1 = 1.5; 
        goal.theta1 = 3.14159/2; 

        goal.x2 = -0.5; 
        goal.y2 = 0; 
        goal.theta2 = -3.14159/2; 

        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 

        while(1)
        {
            if(g_allClear == true)
            {
                ROS_INFO("ALL CLEAR");
            }
            else
            {
                ROS_INFO("OBSTACLE");
                action_client.cancelAllGoals(); 
            }
            ros::spinOnce();
        }

        // bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
    }

    return 0;
}

