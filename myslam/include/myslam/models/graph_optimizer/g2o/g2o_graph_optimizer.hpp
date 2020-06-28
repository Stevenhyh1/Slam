#ifndef MYSLAM_MODELS_GRAPH_OPTIMIZER_G2O_G2O_GRAPH_OPTIMIZER_HPP_
#define MYSLAM_MODELS_GRAPH_OPTIMIZER_G2O_G2O_GRAPH_OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "myslam/models/graph_optimizer/g2o/edge/edge_se3_priorquat.hpp"
#include "myslam/models/graph_optimizer/g2o/edge/edge_se3_priorxyz.hpp"
#include "myslam/models/graph_optimizer/graph_optimizer_interface.hpp"

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(pcg);
G2O_USE_OPTIMIZATION_LIBRARY(csparse);

namespace myslam {
class G2OGraphOptimizer: public GraphOptimizerInterface                             {
public:
    G2OGraphOptimizer(const std::string &solver_type = "lm_var");
    // Optimization
    bool Optimize() override;
    // Set Input and Output
    bool GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) override;
    int GetNodeNum() override;
    // Add edges, vertices, kernels
    void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
    void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) override;
    void AddSe3Edge(int vertex_index1, int vertex_index2, const Eigen::Isometry3d &relative_pose, const Eigen::VectorXd noise) override;
    void AddSe3PriorXYZEdge(int se3_vertex_index, const Eigen::Vector3d &xyz, Eigen::VectorXd noise) override;
    void AddSe3PrioQuaternionEdge(int se3_vertex_index, const Eigen::Quaterniond &quat, Eigen::VectorXd noise) override;
private:
    Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
    Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise);
    Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
    void AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size);

    g2o::RobustKernelFactory *robust_kernel_factory_;
    std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

    std::string robust_kernel_name_;
    double robust_kernel_size_;
    bool need_robust_kernel_ = false;
};
}

#endif