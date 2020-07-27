#include <cmath>
#include <iostream>

#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <glog/logging.h>

struct F1
{
    template <typename T>
    bool operator() (const T* const x1, const T* const x2, T* residual) const {
        residual[0] = x1[0] + 10.0 * x2[0];
        return true;
    }
};

struct F2
{
    template <typename T>
    bool operator() (const T* const x3, const T* const x4, T* residual) const {
        residual[0] = std::sqrt(5) * (x3[0] - x4[0]);
        return true;
    }
};

struct F3
{
    template <typename T>
    bool operator() (const T* const x2, const T* const x3, T* residual) const {
        residual[0] = (x2[0] - 2.0*x3[0]) * (x2[0] - 2.0*x3[0]);
        return true;
    }
};


struct F4
{
    template <typename T>
    bool operator() (const T* const x1, const T* const x4, T* residual) const {
        residual[0] = std::sqrt(10) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
        return true;
    }
};


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    // inital values
    double x1 = 3.0, x2 = -1.0, x3 = 0, x4 = 1.0;

    // ceres problem
    ceres::Problem problem;

    // Cost function
    ceres::CostFunction *c1 = new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1);
    ceres::CostFunction *c2 = new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2);
    ceres::CostFunction *c3 = new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3);
    ceres::CostFunction *c4 = new ceres::AutoDiffCostFunction<F4, 1, 1, 1>(new F4);

    problem.AddResidualBlock(c1, NULL, &x1, &x2);
    problem.AddResidualBlock(c2, NULL, &x3, &x4);
    problem.AddResidualBlock(c3, NULL, &x1, &x4);
    problem.AddResidualBlock(c4, NULL, &x1, &x4);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << " x1: " << x1 
              << " x2: " << x2
              << " x3: " << x3
              << " x4: " << x4 << std::endl;
}