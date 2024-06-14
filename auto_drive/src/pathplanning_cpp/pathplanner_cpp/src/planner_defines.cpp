//
// Created by emile on 24/05/26.
//

#include <planner_defines.hpp>
#include <util_functions.hpp>
#include <cmath>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


namespace ob = ompl::base;


XY rot2D(XY xy, double theta){
    auto [x, y] = xy;
    return std::make_tuple(x * std::cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta));
}
XY operator+ (const XY& a, const XY& b){
    return {std::get<0>(a) + std::get<0>(b), std::get<1>(a) + std::get<1>(b)};
}

XY operator- (const XY& a, const XY& b){
    return {std::get<0>(a) - std::get<0>(b), std::get<1>(a) - std::get<1>(b)};
}

XY operator* (const XY& a, double b){
    return {std::get<0>(a) * b, std::get<1>(a) * b};
}

XY operator* (double a, const XY& b){
    return {a * std::get<0>(b), a * std::get<1>(b)};
}

double length(const XY& a, const XY& b){
    return std::sqrt(std::pow(std::get<0>(a) - std::get<0>(b), 2.0) + std::pow(std::get<1>(a) - std::get<1>(b), 2.0));
}

double product(const XY& a, const XY& b){
    return std::get<0>(a) * std::get<0>(b) + std::get<1>(a) * std::get<1>(b);
}


std::tuple<double, double, double> FK(const ob::State* state){  // r, theta, phi
    const auto *r_vec = state->as<ob::RealVectorStateSpace::StateType>();
    const double *tmp = r_vec->values;
    auto [theta, r, phi] = std::make_tuple(tmp[0], tmp[1], tmp[2]);
    return std::make_tuple(r*cos(theta), r*sin(theta), phi - theta);
}

std::tuple<double, double> get2DPos(const ompl::base::State *state) {
    const auto *r_vec = state->as<ob::RealVectorStateSpace::StateType>();
    const double *tmp = r_vec->values;
    auto [a, b] = std::make_tuple(tmp[0], tmp[1]);
    return std::make_tuple(a, b);
}

double dist_point_to_segment(const XY& point, const XY& l1, const XY& l2){
    assert(length(l1, l2) > 0.0);
    auto [x, y] = closest_point_on_segment(point, l1, l2);
    return length(point, {x, y});
}

XY closest_point_on_segment(const XY& point, const XY& l1, const XY& l2){
    assert(length(l1, l2) > 0.0);
    double prod = product(l2 - l1, point - l1) / length(l1, l2);
    double len = clip(prod, 0.0, length(l1, l2)) / length(l1, l2);
    return l1 + (l2 - l1) * len;
}


double CircleSpace::clearance(const XY &point, double offset) const {
    if (this->is_fill()){
        return length(_center, point) - _r - offset;
    }else{
        return _r - offset - length(_center, point);
    }
}

bool CircleSpace::check_valid(const XY &p1, const XY &p2) const {
    double dist = dist_point_to_segment(_center, p1, p2);
    if(this->is_fill()){
        return dist > _r;
    }else{
        return dist < _r;
    }
}

std::pair<bool, std::optional<double>> CircleSpace::check_valid_and_dist(const XY &p1, const XY &p2) const {
    bool flag = this->check_valid(p1, p2);
    if(!flag)return {false, std::nullopt};  // invalidならそのまま返す

    double dist = dist_point_to_segment(_center, p1, p2);
    return {true, this->is_fill() ? dist - _r : _r - dist};
}



RectangleSpace::RectangleSpace(XY xy_min, XY xy_max, bool fill) : SpaceShapes(fill), _xy_min(xy_min), _xy_max(xy_max){
    this->_vertex = {
        _xy_max,
        {std::get<0>(_xy_min), std::get<1>(_xy_max)},
        _xy_min,
        {std::get<0>(_xy_max), std::get<1>(_xy_min)}
    };

    this->_edges = {
        {this->_vertex[0], this->_vertex[1]},
        {this->_vertex[1], this->_vertex[2]},
        {this->_vertex[2], this->_vertex[3]},
        {this->_vertex[3], this->_vertex[0]}
    };
}


bool RectangleSpace::_check_in_the_rectangle(const XY &point) const {
    auto& [x, y] = point;
    auto& [x_min, y_min] = _xy_min;
    auto& [x_max, y_max] = _xy_max;
    return (x_min <= x && x <= x_max && y_min <= y && y <= y_max);
}


double RectangleSpace::clearance(const XY &point, double offset) const {
    auto [x_min, y_min] = _xy_min;
    auto [x_max, y_max] = _xy_max;
    auto [x, y] = point;

    if (this->is_fill()){
        return std::max({x_min - x, x - x_max, y_min - y, y - y_max}) - offset;
    }else{
        return std::min({x - x_min, x_max - x, y - y_min, y_max - y}) - offset;
    }
}

bool RectangleSpace::check_valid(const XY &p1, const XY &p2) const {
    if(this->is_fill()) {
        // 各コーナーの頂点から最短距離となる線分上の点が長方形に含まれるならfalse
        return std::none_of(_vertex.begin(), _vertex.end(), [this, &p1, &p2](auto &vertex) {
            return (this->_check_in_the_rectangle(closest_point_on_segment(vertex, p1, p2)));
        });
    }else{
        // 線分の端点が長方形に含まれるならtrue
        return (this->_check_in_the_rectangle(p1) && this->_check_in_the_rectangle(p2));
    }
}



std::pair<bool, std::optional<double>> RectangleSpace::check_valid_and_dist(const XY &p1, const XY &p2) const {
    bool flag = this->check_valid(p1, p2);
    if(!flag)return {false, std::nullopt};  // invalidならそのまま返す

    // 最短距離を求める
    double min_dist = std::numeric_limits<double>::max();
    if (this->is_fill()) {
        for(auto& vertex : _vertex){
            chmin(min_dist, dist_point_to_segment(vertex, p1, p2));
        }
        for(const auto& edge : _edges){
            chmin(min_dist, dist_point_to_segment(p1, std::get<0>(edge), std::get<1>(edge)));
            chmin(min_dist, dist_point_to_segment(p2, std::get<0>(edge), std::get<1>(edge)));
        }
    }else{
        for(auto& edge : _edges){
            chmin(min_dist, dist_point_to_segment(p1, std::get<0>(edge), std::get<1>(edge)));
            chmin(min_dist, dist_point_to_segment(p2, std::get<0>(edge), std::get<1>(edge)));
        }
    }
    return {true, min_dist};
}

