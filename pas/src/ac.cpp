// Path Action Client
// Written by Trent Ziemer 2/21/2017, heavily based on work from Dr. Wyatt Newman

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// #include <example_action_server/demoAction.h>
#include <pas/moveAction.h>

void doneCb(const actionlib::SimpleClientGoalState& state,
        const pas::moveResultConstPtr& result) {
    ROS_INFO(" DoneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("Result output from doneCb = %d; goal_stamp = %d; diff = %d", result->output, result->goal_stamp, diff);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_client"); // name this node 
    int g_count = 0;

    // here is a "goal" object compatible with the server
    pas::moveGoal goal;

    // use the name of our server, which is: example_action (named in pas.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
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

    ROS_INFO("Client has connected to action server"); // if here, then we connected to the server;

    while (true) {
        g_count++;
        goal.input = g_count; // this merely sequentially numbers the goals sent
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("Client is giving up waiting on result for goal number %d", g_count);
            return 0;
        } else {
            //if here, then server returned a result to us
        }

    }

    return 0;
}

