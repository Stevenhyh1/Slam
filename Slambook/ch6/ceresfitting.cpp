#include <iostream>
#include <ceres/ceres.h>
#include <chrono>
#include <vector>
#include <opencv2/core/core.hpp>

using namespace std;

struct  CURVE_FITTING_COST
{
    CURVE_FITTING_COST( double x, double y): _x(x), _y(y) {}
    template <typename T>
    bool operator() (const T* const abc, T* residual) const {
        residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
        return true;
    }
    const double _x, _y;
};


int main(int argc, char *argv[]) {
    double a=1.0, b=2.0, c=1.0;
    int N=100;
    double w_sigma=1.0;
    cv::RNG rng;
    double abc[3] = {0,0,0}; //Cannot use vector here, only array

    vector<double> x_data, y_data;
    for (int i=0; i<N; i++) {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(
            exp(a*x*x + b*x + c) + rng.gaussian(w_sigma)
        );
        cout << x_data[i] << " " <<  y_data[i] << endl;
    }

    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
        problem.AddResidualBlock(     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
            new CURVE_FITTING_COST(x_data[i], y_data[i])
        ),
        nullptr,            // 核函数，这里不使用，为空
        abc);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve (options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2-t1);
    cout << "solve time cost = " << time_used.count() << "seconds. " << endl;

    cout << summary.BriefReport() << endl;
    cout << "estimated a, b, c = ";
    for (auto i:abc) cout << i << " ";
    cout << endl;

    return 0;
}