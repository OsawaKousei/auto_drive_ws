#ifndef F7_SIM_UTIL_FUNCTIONS_HPP
#define F7_SIM_UTIL_FUNCTIONS_HPP

#include <functional>

template<class T>bool chmax(T &former, const T &b) { if (former<b) { former=b; return true; } return false; }
template<class T>bool chmin(T &former, const T &b) { if (b<former) { former=b; return true; } return false; }

template<class T>T clip(const T& value, const T& min, const T& max){
    return std::min(max, std::max(min, value));
}

template<class T>bool chclip(T& ans,const T& value, const T& min, const T& max){
    ans = std::min(max, std::max(min, value));
    if(value < min || value > max) return true;
    else return false;
}

#endif //F7_SIM_UTIL_FUNCTIONS_HPP

