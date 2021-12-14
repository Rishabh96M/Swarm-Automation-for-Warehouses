/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief site class executable
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

#include <cmath>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include "../../include/swarm_master/site.hpp"

/**
 * @brief Calculate squared distances between all robots ad individual site
 *
 * @param robots
 */
void Site::populate_robot_dists(std::unordered_map<int, Robot>& robots) {
    for (const auto& robot : robots) {
        double dist_sq = pow(crate.start_pos[0], robot.second.pos[0]) +
        pow(crate.start_pos[1], robot.second.pos[1]);
        dist_to_robots.push_back({robot.first, dist_sq});
    }
    std::sort(dist_to_robots.begin(), dist_to_robots.end());
}

/**
 * @brief Constructor for site
 *
 * @param id
 */
std::vector<RobotDist> Site::get_n_closest(int n) {
    if (n == -1) {
        return std::vector<RobotDist>(dist_to_robots.begin(),
        dist_to_robots.end());
    } else if (n >= dist_to_robots.size()) {
      throw std::invalid_argument("Not enough robots");
    } else {
        return std::vector<RobotDist>(dist_to_robots.begin(),
        dist_to_robots.begin()+n);
      }
    }
