//
// Created by emile on 24/05/08.
//

#ifndef PLANNER_CPP_UTIL_FUNCTIONS_HPP
#define PLANNER_CPP_UTIL_FUNCTIONS_HPP

#include <functional>

template<class T>bool chmax(T &former, const T &b) { if (former<b) { former=b; return true; } return false; }
template<class T>bool chmin(T &former, const T &b) { if (b<former) { former=b; return true; } return false; }

template<class T>T clip(const T& value, const T& min, const T& max){
    return std::min(max, std::max(min, value));
}

double convert_angle_in_pi(double angle_rad);

int binary_search(int n_min, int n_max, std::function<bool(int)> f);  // fはleftでfalse, rightでtrue


#endif //CPPCODES_UTIL_FUNCTIONS_HPP
