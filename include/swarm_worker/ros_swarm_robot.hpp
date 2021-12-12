/**
 * @file RosSwarmRobot.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief RosSwarmRobot class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <ros.h>
#include <string>
#include <array>
#include "./swarm_robot.hpp"
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <warehouse_swarm/RobotTask.h>


class RosSwarmRobot : public SwarmRobot {
 private:
    ros::NodeHandle nh;
    std::string task_service_topic;
    std::string pos_publisher_topic;
    ros::ServiceClient swarm_connect_client;
    ros::ServiceServer task_server;
    ros::Publisher pos_publisher;
    ros::Publisher vel_pub;
    ros::Subscriber pos_sub;
    ros::Subscriber task_sub;
    bool ready = false;

 public:
    RosSwarmRobot(std::string task_service_topic);
    ~RosSwarmRobot();

    /**
     * @brief Connect to swarm master
     * 
     * @return true 
     * @return false 
     */
    bool connect_to_master();

    /**
     * @brief Publish robot position
     * 
     * @return true 
     * @return false 
     */
    void publish_robot_pos();

    /**
     * @brief Drive with mecanum wheels
     * 
     * @return true 
     * @return false 
     */
    bool drive_mecanum(std::array<double, 2>);

    /**
     * @brief Turn with mecanum wheels
     * 
     * @return true 
     * @return false 
     */
    bool turn_mecanum(double);

    /**
     * @brief Set the platform height
     * 
     * @return true 
     * @return false 
     */
    bool publish_platform_height(double);
    
    void update_robot_pos(const nav_msgs::Odometry::ConstPtr& msg);
    
    void get_task_callback(const warehouse_swarm::RobotTask::ConstPtr& task_msg);

    void wait(int site_id);

    void is_ready(std_msgs::Empty::ConstPtr&);

    void run();
};
