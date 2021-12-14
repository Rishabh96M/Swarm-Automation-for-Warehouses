/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief AssignmentDesignator class executable
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

#include <map>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <unordered_map>
#include "../../include/swarm_master/assignment_designator.hpp"

/**
  * @brief Method definition for get_designations
  *
  * @param all_sites - pointer to unordered_map<int, Site>
  * @param all_robots - pointer to unordered_map<int, Robot>
  * @return virtual SiteVec
  */
SimpleClosestDesignator::SiteVec SimpleClosestDesignator::get_designations(
        std::unordered_map<int, Site>& all_sites,
        std::unordered_map<int, Robot>& all_robots) {
    SiteVec ret = std::make_shared<std::vector<Site> >();
    int robots_required{0};
    std::unordered_map<int, int> used_id_map{};
    for (auto& site_pair : all_sites) {
        auto& site = site_pair.second;
        site.populate_robot_dists(all_robots);
        for (const auto& robot : site.get_n_closest(-1)) {
          if (used_id_map.find(robot.robot_id) != used_id_map.end()) continue;
          if (site.assigned_ids.size() == site.robots_required) break;
          used_id_map[robot.robot_id] = site.site_id;
          site.assigned_ids.push_back(robot.robot_id);
        }
        if (site.assigned_ids.size() != site.robots_required)
            throw std::length_error("Error finding robot assignments");
    }

    for (const auto& used_id : used_id_map) {
        auto& site = all_sites.at(used_id.second);
        if (site.assigned_ids.size() < site.robots_required) {
            all_sites.at(used_id.second).assigned_ids.push_back(used_id.first);
        }
    }

    for (auto site : all_sites)
        ret->push_back(site.second);

    return ret;
}
