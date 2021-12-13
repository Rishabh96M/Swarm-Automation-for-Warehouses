/**
 * @file main.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Main script for RosSwarmMaster
 * @version 0.1
 * @date 2021-12-06
 *
 * @copyright Copyright (c) 2021 TBD
 *
 */

#include <ros/ros.h>
#include "../../include/swarm_master/ros_swarm_master.hpp"
#include "../../include/swarm_master/assignment_designator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_master");
    SimpleClosestDesignator designator;
    RosSwarmMaster master(&designator, 2.0);
    auto* simple_master = master.get_swarm_master();

    simple_master->add_crate_to_system(Crate({5.0, 0.0, 0.25}, {-5.0, 0.0, 0.25}, {0.6, 0.6}, 2));
    // simple_master->add_crate_to_system(Crate({5.0, 2.0, 0.25}, {-5.0, 2.0, 0.25}, {0.6, 0.6}, 7));
    // simple_master->add_crate_to_system(Crate({5.0, 4.0, 0.25}, {-5.0, 4.0, 0.25}, {0.6, 0.6}, 5));
    // simple_master->add_crate_to_system(Crate({5.0, 6.0, 0.25}, {-5.0, 6.0, 0.25}, {0.6, 0.6}, 2));
    // simple_master->add_crate_to_system(Crate({5.0, -2.0, 0.25}, {-5.0, -2.0, 0.25}, {0.6, 0.6}, 7));
    // simple_master->add_crate_to_system(Crate({5.0, -4.0, 0.25}, {-5.0, -4.0, 0.25}, {0.6, 0.6}, 5));
    // simple_master->add_crate_to_system(Crate({5.0, -6.0, 0.25}, {-5.0, -6.0, 0.25}, {0.6, 0.6}, 2));

    simple_master->add_robot_to_swarm({0,2});

    simple_master->add_robot_to_swarm({1,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    // simple_master->add_robot_to_swarm({0,2});
    master.startup();
    ros::spin();
    return 0;
}
