/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief Task struct declaration
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

#include <string>
#include <unordered_map>

/**
  * @brief Structure definition fot Task
  */
struct Task {
    enum taskType {
        Drive,
        MvPlatform,
        Wait,
    };

    /**
     * Variable Declarations
     */
    taskType task;
    std::unordered_map<std::string, double> num_param_dict;

    /**
      * @brief Constructor for struct Task
      *
      * @param task_type - struct with Drive, MvPlatform and Wait
      * @param command_dict - std::unordered_map<std::string, double>
      */
    Task(taskType task_type,
      std::unordered_map<std::string, double> command_dict) {
        task = task_type;
        num_param_dict = command_dict;
    }
};
