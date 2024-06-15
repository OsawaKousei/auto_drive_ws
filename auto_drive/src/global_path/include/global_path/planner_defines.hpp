
// Created by emile on 24/05/26.
//

#ifndef PLANNER_CPP_PLANNER_DEFINES_HPP
#define PLANNER_CPP_PLANNER_DEFINES_HPP

#include <optional>
#include <vector>
#include <tuple>
#include <ompl/base/State.h>

namespace global_path{
using XY = std::tuple<double, double>;
XY rot2D(XY xy, double theta);
XY operator+ (const XY& a, const XY& b);
XY operator- (const XY& a, const XY& b);
XY operator* (const XY& a, double b);
XY operator* (double a, const XY& b);
double length(const XY& a, const XY& b);
double product(const XY& a, const XY& b);

std::tuple<double, double, double> FK(const ompl::base::State* state);
std::tuple<double, double> get2DPos(const ompl::base::State* state);

double dist_point_to_segment(const XY& point, const XY& l1, const XY& l2);
XY closest_point_on_segment(const XY& point, const XY& l1, const XY& l2);


enum PlannerType{  // Ubuntu22.04ならもっと多くの種類が使えるはず
    PLANNER_BFMTSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};

struct RobotState{
    double x, y, theta;
    RobotState(): x(0), y(0), theta(0){};
    RobotState(double x, double y, double theta): x(x), y(y), theta(theta){};
};


class SpaceShapes {
    bool _fill;
public:
    explicit SpaceShapes(bool fill): _fill(fill){};
    virtual double clearance(const XY& point, double offset) const = 0;
    [[nodiscard]] virtual bool check_valid(const XY& p1, const XY& p2) const = 0;
    [[nodiscard]] virtual std::pair<bool, std::optional<double>> check_valid_and_dist(const XY& p1, const XY& p2) const = 0;
    // 線分p1, p2との衝突判定とobjectとの最短距離を返す (衝突してる場合は{false, std::nullopt}を返す)

    [[nodiscard]] bool is_fill() const { return _fill; }
    virtual ~SpaceShapes() = default;
};


class CircleSpace : public SpaceShapes{
    XY _center;
    double _r;
public:
    CircleSpace(XY center, double r, bool fill) : SpaceShapes(fill), _center(center), _r(r) {};
    double clearance(const XY& point, double offset = 0.0) const override;
    [[nodiscard]] bool check_valid(const XY& p1, const XY& p2) const;
    [[nodiscard]] std::pair<bool, std::optional<double>> check_valid_and_dist(const XY& p1, const XY& p2) const override;
};


class RectangleSpace : public SpaceShapes{
    XY _xy_min, _xy_max;
    std::vector<XY> _vertex;
    std::vector<std::tuple<XY, XY>> _edges;
    [[nodiscard]] bool _check_in_the_rectangle(const XY& point) const;
public:
    RectangleSpace(XY xy_min, XY xy_max, bool fill);
    double clearance(const XY& point, double offset = 0.0) const override;
    [[nodiscard]] bool check_valid(const XY& p1, const XY& p2) const;
    [[nodiscard]] std::pair<bool, std::optional<double>> check_valid_and_dist(const XY& p1, const XY& p2) const override;
};
}


#endif