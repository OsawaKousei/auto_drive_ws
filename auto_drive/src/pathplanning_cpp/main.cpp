//
// Created by emile on 24/05/11.
//

#include <matplotlibcpp17/pyplot.h>
#include <bits/stdc++.h>
#include "pathplanner_cpp/inc/spline_utils.hpp"
#include "pathplanner_cpp/inc/planning.hpp"
#include <pybind11/pybind11.h>
#include <iostream>
#include <chrono>


int main() {
    std::chrono::system_clock::time_point start_time, end_time;
    pybind11::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    auto patches = pybind11::module::import("matplotlib.patches");
    namespace py = pybind11;

    std::vector<double> xs = {1, 2, 3, 4};
    std::vector<double> ys = {1, -4, 9, -16};

    RobotState start(0, 0, 0);
    RobotState goal(10, 10, 0);
    OMPL_PlannerClass planner;

    start_time = std::chrono::system_clock::now();
    std::vector<RobotState> global_path = planner.plan(start, goal);
    end_time = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換
    std::cout << "planning time: " << elapsed << " ms" << std::endl;

    xs.clear();
    ys.clear();

    for (auto& state : global_path) {
        xs.push_back(state.x);
        ys.push_back(state.y);
    }


    start_time = std::chrono::system_clock::now();
    auto [xs_new, ys_new] = spline_by_num(xs, ys, 100);  // スプライン補間
    end_time = std::chrono::system_clock::now();
    double elapsed_first = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count(); //処理に要した時間をミリ秒に変換

    start_time = std::chrono::system_clock::now();
    auto [xs_local, ys_local] = spline_by_min_max(xs, ys, 0.1, 1.5, 0.15);  // 台形加減速
    end_time = std::chrono::system_clock::now();
    double elapsed_second = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count();

    std::cout << "elapsed time for spline_by_num: " << elapsed_first << " ms" << std::endl;
    std::cout << "elapsed time for spline_by_min_max: " << elapsed_second << " ms" << std::endl;

    auto [fig, ax] = plt.subplots();
    ax.plot(Args(xs_new, ys_new),Kwargs("color"_a = "blue"));
    ax.plot(Args(xs, ys),Kwargs("color"_a = "red", "linestyle"_a = "dashed"));
    ax.plot(Args(xs_local, ys_local),Kwargs("color"_a = "cyan", "linestyle"_a = "dotted", "marker"_a = "o"));

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