//
// Created by emile on 24/05/06.
//

#ifndef PLANNER_CPP_SPLINE_UTILS_HPP
#define PLANNER_CPP_SPLINE_UTILS_HPP

#include <vector>
#include "planner_defines.hpp"


std::pair<std::vector<double>, std::vector<double>> spline_by_num(std::vector<double> xs, std::vector<double> ys, const int& num_points);
std::pair<std::vector<double>, std::vector<double>> spline_by_min_max(std::vector<double> xs, std::vector<double> ys,
                                                                      double l_min, double l_max, double d_max);




#endif //CPPCODES_SPLINE_UTILS_HPP
