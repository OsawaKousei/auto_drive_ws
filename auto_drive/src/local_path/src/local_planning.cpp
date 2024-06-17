//
// Created by emile on 24/05/06.
//

#include <local_path/local_planning.hpp>
#include <spline.hpp>
// #include <util_functions.hpp>
#include <cassert>
#include <cmath>


void create_time_grid(std::vector<double>& T, double& tmin, double& tmax,
                      std::vector<double>& X, std::vector<double>& Y, bool is_closed_curve){
    assert(X.size()==Y.size() && X.size()>2);

    // hack for closed curves (so that it closes smoothly):
    //  - append the same grid points a few times so that the spline
    //    effectively runs through the closed curve a few times
    //  - then we only use the last loop
    //  - if periodic boundary conditions were implemented then
    //    simply setting periodic bd conditions for both x and y
    //    splines is sufficient and this hack would not be needed
    int idx_first=-1, idx_last=-1;
    if(is_closed_curve) {
        // remove last point if it is identical to the first
        if(X[0]==X.back() && Y[0]==Y.back()) {
            X.pop_back();
            Y.pop_back();
        }

        const int num_loops=3;  // number of times we go through the closed loop
        std::vector<double> Xcopy, Ycopy;
        for(int i=0; i<num_loops; i++) {
            Xcopy.insert(Xcopy.end(), X.begin(), X.end());
            Ycopy.insert(Ycopy.end(), Y.begin(), Y.end());
        }
        idx_last  = (int)Xcopy.size()-1;
        idx_first = idx_last - (int)X.size();
        X = Xcopy;
        Y = Ycopy;

        // add first point to the end (so that the curve closes)
        X.push_back(X[0]);
        Y.push_back(Y[0]);
    }

    // setup a "time variable" so that we can interpolate x and y
    // coordinates as a function of time: (X(t), Y(t))
    T.resize(X.size());
    T[0]=0.0;
    for(size_t i=1; i<T.size(); i++) {
        // time is proportional to the distance, i.e. we go at a const speed
        T[i] = T[i-1] + sqrt( pow(X[i]-X[i-1], 2.0) + pow(Y[i]-Y[i-1], 2.0) );
    }
    if(idx_first<0 || idx_last<0) {
        tmin = T[0] - 0.0;
        tmax = T.back() + 0.0;
    } else {
        tmin = T[idx_first];
        tmax = T[idx_last];
    }
}

std::pair<std::vector<double>, std::vector<double>> spline_by_num(std::vector<double> xs, std::vector<double> ys, const int &num_points) {
    assert(xs.size()==ys.size() && xs.size()>2);

    // parametric spline
    tk::spline::spline_type line_type = tk::spline::cspline;

    bool is_closed = xs[0]==xs.back() && ys[0]==ys.back();
    double t_min = 0.0, t_max = 0.0;
    std::vector<double> times; // parameter
    create_time_grid(times, t_min, t_max, xs, ys, is_closed);

    tk::spline spline_xs, spline_ys;
    spline_xs.set_points(times, xs, line_type);
    spline_ys.set_points(times, ys, line_type);

    std::vector<double> new_xs, new_ys;
    for(int i=0; i<num_points; i++) {
        double t = t_min + (t_max-t_min) * i / (num_points-1);
        new_xs.push_back(spline_xs(t));
        new_ys.push_back(spline_ys(t));
    }
    return std::make_pair(new_xs, new_ys);
}




