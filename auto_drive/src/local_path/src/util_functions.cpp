//
// Created by emile on 24/05/08.
//

#include <util_functions.hpp>
#include <cmath>

double convert_angle_in_pi(double angle_rad){
    while(angle_rad >= M_PI)angle_rad -= 2.0*M_PI;
    while(angle_rad < -M_PI)angle_rad += 2.0*M_PI;
    return angle_rad;
}

int binary_search(int n_min, int n_max, std::function<bool(int)> f) {  // fはleftでfalse, rightでtrue
    int left = n_min-1;  // 常に f(left) = false
    int right = n_max; // 常に f(right) = true

    while (right - left > 1) {  // meguru
        int mid = left + (right - left) / 2;

        if (f(mid)){
            right = mid;
        } else {
            left = mid;
        }
    }
    return right; // left は条件を満たさない最大の値、right は条件を満たす最小の値になる
}

