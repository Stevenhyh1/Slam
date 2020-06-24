#include "myslam/models/graph_optimizer/graph_optimizer_interface.hpp"

namespace myslam {
void GraphOptimizerInterface::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}// namespace myslam