#include "global_path/global_planning.hpp"

#include <matplotlibcpp17/pyplot.h>
#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <iostream>
#include <chrono>

#include <geometry_msgs/msg/pose.hpp>

int main() {
    std::chrono::system_clock::time_point start_time, end_time;
    pybind11::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    auto patches = pybind11::module::import("matplotlib.patches");
    namespace py = pybind11;

    std::vector<double> xs = {1, 2, 3, 4};
    std::vector<double> ys = {1, -4, 9, -16};

    auto start = geometry_msgs::msg::Pose();
    auto goal = geometry_msgs::msg::Pose();
  
    global_path::OMPL_PlannerClass planner;

    start_time = std::chrono::system_clock::now();
    nav_msgs::msg::Path global_path = planner.plan(start, goal);
    end_time = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換
    std::cout << "planning time: " << elapsed << " ms" << std::endl;

    xs.clear();
    ys.clear();

    for (auto& state : global_path.poses) {
        xs.push_back(state.pose.position.x);
        ys.push_back(state.pose.position.y);
    }

    auto [fig, ax] = plt.subplots();
    ax.plot(Args(xs, ys),Kwargs("color"_a = "red", "linestyle"_a = "dashed"));

    auto rect = patches.attr("Rectangle")(
            py::make_tuple(2.0, 2.0), 2.0, 2.0,
            py::arg("edgecolor") = "black",
            py::arg("facecolor") = "black"
    );

    auto circ = patches.attr("Circle")(
            py::make_tuple(7.0, 7.0), 2.0,
            py::arg("edgecolor") = "black",
            py::arg("facecolor") = "black"
    );

    ax.add_patch(Args(rect));
    ax.add_patch(Args(circ));
    ax.set_aspect(Args("equal"));
    plt.show();
}