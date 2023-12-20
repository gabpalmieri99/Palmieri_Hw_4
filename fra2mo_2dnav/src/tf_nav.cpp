#include "../include/tf_nav.h"

TF_NAV::TF_NAV()
{
    _selected_goal = "homepos";
    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>("/fra2mo/pose", 1);
    _trajectory_pub_x = _nh.advertise<std_msgs::Float64>("/robot_trajectory_x", 1);
    _trajectory_pub_y = _nh.advertise<std_msgs::Float64>("/robot_trajectory_y", 1);

    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;
    _goal_pos << 0.0, 0.0, 0.0;
    _goal_or << 0.0, 0.0, 0.0, 1.0;
    _home_pos << -3.0, 5.0, 0.0;
}

void TF_NAV::tf_listener_fun()
{
    ros::Rate r(5);
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while (ros::ok())
    {
        try
        {
            listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }
}

void TF_NAV::position_pub()
{
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];
    
    std_msgs::Float64 robot_x, robot_y;
    robot_x.data = _cur_pos[0];
    robot_y.data = _cur_pos[1];  

    _position_pub.publish(pose);
     
    _trajectory_pub_x.publish(robot_x);
    _trajectory_pub_y.publish(robot_y);

}

void TF_NAV::goal_listener()
{
    ros::Rate r(1);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    bool hasSlept=false;

    while (ros::ok())
    {

        try
        {
            listener.waitForTransform("map", _selected_goal, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", _selected_goal, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or << transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        
         // DEBUG
                if (!hasSlept)
                {
                    ros::Duration(10.0).sleep(); // Sleep once
                    hasSlept = true;             // Set the flag to true after sleeping
                }

                tf::Quaternion quaternion;
                quaternion = tf::Quaternion(
                    _goal_or[1],
                    _goal_or[2],
                    _goal_or[3],
                    _goal_or[0]);

                tf::Matrix3x3 q(quaternion);
                double roll, pitch, yaw;
                q.getRPY(roll, pitch, yaw);
                roll = roll * 180 / 3.14;
                pitch = pitch * 180 / 3.14;
                yaw = yaw * 180 / 3.14;

                std::cout << "Position (x, y, z): ("
                          << _goal_pos[0] << ", "
                          << _goal_pos[1] << ", "
                          << _goal_pos[2] << ")" << std::endl;
                std::cout << "Orientation (r, p, y): "         
                           <<"Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

        r.sleep();
    }
}

void TF_NAV::get_goals()
{

    int input = 0;
    std::cout << "Please insert the number of desired goals: ";
    std::cin >> _goal_number;
    std::cout << "Please insert the order of the " << _goal_number << " desired goals: ";
    for (int i = 0; i < _goal_number; ++i)
    {
        std::cin >> input;
        goalOrder.push_back(input);
    }

    std::cout << "Contents of goalOrder vector: ";
    for (int i : goalOrder)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    _selected_goal = "goal" + std::to_string(goalOrder[0]);
}

void TF_NAV::send_goal()
{
    ros::Rate r(5);
    int cmd;
    bool hasSlept=false;
    move_base_msgs::MoveBaseGoal goal;

    while (ros::ok())
    {

        std::cout << "\nInsert 1 to send goal from TF " << std::endl;
        std::cout << "Insert 2 to send home position goal " << std::endl;
        std::cout << "Insert your choice" << std::endl;
        std::cin >> cmd;

        if (cmd == 1)
        {
            int count = 0;
            get_goals();

            if (!hasSlept)
                {
                    ros::Duration(10.0).sleep(); // Sleep once
                    hasSlept = true;             // Set the flag to true after sleeping
                }


            while (count < _goal_number)
            {
                MoveBaseClient ac("move_base", true);
                while (!ac.waitForServer(ros::Duration(5.0)))
                {
                    ROS_INFO("Waiting for the move_base action server to come up");
                }

                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();

                goal.target_pose.pose.position.x = _goal_pos[0];
                goal.target_pose.pose.position.y = _goal_pos[1];
                goal.target_pose.pose.position.z = _goal_pos[2];

                goal.target_pose.pose.orientation.w = _goal_or[0];
                goal.target_pose.pose.orientation.x = _goal_or[1];
                goal.target_pose.pose.orientation.y = _goal_or[2];
                goal.target_pose.pose.orientation.z = _goal_or[3];

                ROS_ERROR("Sending goal");
                           
                ac.sendGoal(goal);
                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {

                    ROS_ERROR("The mobile robot arrived in the TF goal");

                    count = count + 1;
                    _selected_goal = "goal" + std::to_string(goalOrder[count]);
                    ros::Duration(1.0).sleep();
                }

                else
                    ROS_INFO("The base failed to move for some reason");
            }

            if (count == _goal_number)
            {

                ROS_ERROR("The mobile robot arrived correctly to all the TF goals! ");
                goalOrder.clear();
                ros::Duration(5.0).sleep();

            }
        }

        else if (cmd == 2)
        {
            MoveBaseClient ac("move_base", true);
            while (!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_ERROR("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_ERROR("The mobile robot arrived in the HOME position");

            else
                ROS_INFO("The base failed to move for some reason");
        }
        else
        {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
}

void TF_NAV::run()
{
    boost::thread tf_listener_fun_t(&TF_NAV::tf_listener_fun, this);
    boost::thread tf_listener_goal_t(&TF_NAV::goal_listener, this);
    boost::thread send_goal_t(&TF_NAV::send_goal, this);
    ros::spin();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tf_navigation");

    TF_NAV tfnav;
    tfnav.run();

    return 0;
}