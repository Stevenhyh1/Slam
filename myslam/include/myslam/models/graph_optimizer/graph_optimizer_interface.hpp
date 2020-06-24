#ifndef MYSLAM_MODELS_GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_INTERFACE_HPP_
#define MYSLAM_MODELS_GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_INTERFACE_HPP_

#include <deque>
#include <string>

#include <Eigen/Dense>

namespace myslam {
class GraphOptimizerInterface
{
public:
    virtual ~GraphOptimizerInterface() {};
    // Optimization
    virtual bool Optimize() = 0;
    // Set Input and Output
    virtual bool GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) = 0;
    virtual int GetNodeNum() = 0;
    // Add edges, vertices, kernels
    virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) = 0;
    virtual void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) = 0;
    virtual void AddSe3Edge(int vertex_index1, int vertex_index2, const Eigen::Isometry3d &relative_pose, const Eigen::VectorXd noise) = 0;
    virtual void AddSe3PriorXYZEdge(int se3_vertex_index, const Eigen::Vector3d &xyz, Eigen::VectorXd noise) = 0;
    virtual void AddSe3PrioQuaternionEdge(int se3_vertex_index, const Eigen::Quaterniond &quat, Eigen::VectorXd noise) = 0;

    void SetMaxIterationsNum(int max_iterations_num);
    int max_iterations_num_ = 512;
    
};
} //namespace myslam
#endif