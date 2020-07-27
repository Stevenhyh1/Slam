#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <glog/logging.h>

// Automatic differentiation
struct CostFunctor {
    template <typename T>
    bool operator() (const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

// Numeric differentiation
struct NumericDiffCostFunctor {
    bool operator() (const double *const x, double *residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

// Analitial differentiation
class AnaCostFunction : public ceres::SizedCostFunction<1, 1> {
    public:
        virtual ~AnaCostFunction() {};
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
            const double x = parameters[0][0];
            residuals[0] = 10 - x;

            if (jacobians != NULL && jacobians[0] != NULL) {
                jacobians[0][0] = -1;
            }
            return true;
        }
};


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    // inital value
    double initial_x = 5.0;
    double x = initial_x;

    // Build the problem
    ceres::Problem problem;

    // Set up the cost function

    // Automatic Differentialtion

    // CostFunction* cost_function
    // = new AutoDiffCostFunction<MyScalarCostFunctor, 1, 2, 2>(
    //     new MyScalarCostFunctor(1.0));              ^  ^  ^
    //                                                 |  |  |
    //                     Dimension of residual ------+  |  |
    //                     Dimension of x ----------------+  |
    //                     Dimension of y -------------------+

    // parameter x is 1 dimensional, residual 10.0 - x is 1 dimentional
    ceres::CostFunction* cost_function =  new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    //                                                                                 ^  ^
    //                                                                                 |  |
    //                                                     Dimension of residual ------+  |
    //                                                     Dimension of x ----------------+  
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Numerical Differentiation
    // ceres::CostFunction *cost_function = new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(new NumericDiffCostFunctor);
    // problem.AddResidualBlock(cost_function, NULL, &x);

    // Analytical Differentiation
    // ceres::CostFunction *cost_function = new AnaCostFunction;
    // problem.AddResidualBlock(cost_function, NULL, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "x : " << initial_x << " -> " << x << std::endl;

}
