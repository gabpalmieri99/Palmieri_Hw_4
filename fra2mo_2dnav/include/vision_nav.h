#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <vector>
#include <future>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

Eigen::Vector3d aruco_pos;
Eigen::Vector4d aruco_or;
void arucoPoseCallback_broadcaster(const geometry_msgs::PoseStamped & msg);
void arucoPoseCallback(const geometry_msgs::PoseStamped &msg);

class TF_NAV {

    public:
        
        TF_NAV();
        
        void get_goals();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();
        void get_aruco_map();
        void aruco_pose_pub();

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;
        ros::Publisher _aruco_pose_pub;
        ros::Subscriber _aruco_pose_sub;

        bool aruco_goal_sent=false;
        bool aruco_pose_available=false;

        Eigen::Vector3d _home_pos;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        Eigen::Vector3d _goal_pos;
        Eigen::Vector4d _goal_or;

        int _goal_number;
        std::vector<int> goalOrder;
        std::string _selected_goal;

        Eigen::Vector3d _aruco_pos;
        Eigen::Vector4d _aruco_or;
        
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
};
