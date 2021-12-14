/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief site class declaration
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

#pragma once

#include <math.h>
#include <array>
#include <vector>
#include <unordered_map>
#include "../structs/robot.hpp"
#include "../structs/crate.hpp"

struct RobotDist {
    int robot_id;
    double dist_sq;
    RobotDist(int _id, double _dist) {
        this->robot_id = _id;
        this->dist_sq = _dist;
    }
    bool operator<(const RobotDist& d2) {
        return this->dist_sq < d2.dist_sq;
    }
};

class Site {
 public:
    int site_id;
    Crate crate{};
    int robots_required;
    std::vector<int> assigned_ids{};
    std::vector<RobotDist> dist_to_robots{};

    /**
     * @brief Constructor for site
     *
     * @param id
     * @param *crate
     * @param weight_per_robot
     */
    Site(int id, const Crate& _crate, double weight_per_robot) :
            site_id{id} {
        double val = std::ceil(_crate.mass/weight_per_robot);
        if (val < 2) val = 2;
        robots_required = static_cast<int>(val);
        crate = _crate;
    }

    /**
     * @brief Constructor for site
     *
     * @param site
     */
    Site(const Site& site) {
        this->assigned_ids = site.assigned_ids;
        this->crate = site.crate;
        this->dist_to_robots = site.dist_to_robots;
        this->site_id = site.site_id;
        this->robots_required = site.robots_required;
    }

    Site& operator=(Site& site) {
        return site;
    }

    /**
     * @brief Calculate squared distances between all robots ad individual site
     *
     * @param robots
     */
    void populate_robot_dists(std::unordered_map<int, Robot>& robots);

    /**
     * @brief Get the n closest robots to site
     *
     * @param n
     * @return std::vector<RobotDist>
     */
    std::vector<RobotDist> get_n_closest(int n);
};
