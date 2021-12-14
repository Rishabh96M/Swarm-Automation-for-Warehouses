/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief AssignmentDesignator class declaration
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
#include <vector>
#include <memory>
#include <unordered_map>
#include "../structs/robot.hpp"
#include "./site.hpp"

/**
  * @brief Class definition fot AssignmentDesignator
  */
class AssignmentDesignator {
 protected:
    typedef std::shared_ptr<std::vector<Site> > SiteVec;
 public:
    /**
      * @brief Method definition for get_designations
      *
      * @param all_sites - pointer to unordered_map<int, Site>
      * @param all_robots - pointer to unordered_map<int, Robot>
      * @return virtual SiteVec - typedef of std::shared_ptr<std::vector<Site>
      */
    virtual SiteVec get_designations(std::unordered_map<int, Site>& all_sites,
                            std::unordered_map<int, Robot>& all_robots) = 0;
};

class SimpleClosestDesignator : public AssignmentDesignator {
 public:
    /**
      * @brief Method definition for get_designations
      *
      * @param all_sites - pointer to unordered_map<int, Site>
      * @param all_robots - pointer to unordered_map<int, Robot>
      * @return virtual SiteVec
      */
    SiteVec get_designations(std::unordered_map<int, Site>& all_sites,
                             std::unordered_map<int, Robot>& all_robots);
};
