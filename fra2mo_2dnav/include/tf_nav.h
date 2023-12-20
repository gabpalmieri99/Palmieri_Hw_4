#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <vector>
#include <future>

#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

class TF_NAV {

    public:
        
        TF_NAV();
        
        void get_goals();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;
        ros::Publisher _trajectory_pub_x, _trajectory_pub_y; 

        Eigen::Vector3d _home_pos;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        Eigen::Vector3d _goal_pos;
        Eigen::Vector4d _goal_or;
        
        int _goal_number;
        std::vector<int> goalOrder;
        std::string _selected_goal;

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


};