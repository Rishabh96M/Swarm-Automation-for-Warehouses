/**
 * Copyright (c) 2021 Prannoy Namala, Dani Lerner, Rishabh Mukund
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief main file for SwarmMaster class
 * @version 3.0.1
 * @date 2021-12-03
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <ros/ros.h>
#include "../../include/swarm_master/ros_swarm_master.hpp"
#include "../../include/swarm_master/assignment_designator.hpp"

/**
 * @brief main class for swarm Master class
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_master");
    SimpleClosestDesignator designator;
    RosSwarmMaster master(&designator, 2.0);

    master.add_robot_to_swarm({0, 0.5});
    master.add_robot_to_swarm({0, -0.5});
    master.add_robot_to_swarm({0, 1.5});
    master.add_robot_to_swarm({0, -1.5});
    master.add_robot_to_swarm({0, 2.5});
    master.add_robot_to_swarm({0, -2.5});
    master.add_robot_to_swarm({0, 3.5});
    master.add_robot_to_swarm({0, -3.5});
    master.add_robot_to_swarm({0, 4.5});
    master.add_robot_to_swarm({0, -4.5});
    master.add_robot_to_swarm({-1, 0});
    master.add_robot_to_swarm({1, 1});
    master.add_robot_to_swarm({1, -1});
    master.add_robot_to_swarm({1, 2});
    master.add_robot_to_swarm({1, -2});
    master.add_robot_to_swarm({2, 0});
    master.add_robot_to_swarm({2, -1});
    master.add_robot_to_swarm({2, 2});
    master.add_robot_to_swarm({2, 1});
    master.add_robot_to_swarm({2, -2});

    auto assignments = master.startup();
    ros::spin();
    return 0;
}
