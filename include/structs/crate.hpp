/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief Crate struct declaration
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

#include <array>

/**
  * @brief Structure definition fot Crate
  */
struct Crate {
   /**
    * Variable Declarations
    */
    double mass;                              // kg
    std::array<double, 3> start_pos;          // m
    std::array<double, 3> goal_pos;           // m
    std::array<double, 2> base_footprint;     // m

    Crate() : start_pos{}, goal_pos{}, base_footprint{}, mass{} {}

    /**
      * @brief Constructor for struct Crate
      *
      * @param start_pos - std::array<double, 3> (x,y,z in m)
      * @param goal_pos - std::array<double, 3>  (x,y,z in m)
      * @param base_footprint - std::array<double, 2> (x,y in m)
      * @param mass - double (mass in kg)
      */
    Crate(std::array<double, 3> _start_pos, std::array<double, 3> _goal_pos,
            std::array<double, 2> _base_footprint, double _mass) :
        mass{_mass},
        start_pos{_start_pos},
        goal_pos{_goal_pos},
        base_footprint{_base_footprint} {}
};
