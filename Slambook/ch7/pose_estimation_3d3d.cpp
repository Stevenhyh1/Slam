#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sophus/se3.hpp>

// Find Matches Features
void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
                          std::vector<cv::KeyPoint> &keypoints_1, 
                          std::vector<cv::KeyPoint> &keypoints_2,
                          std::vector<cv::DMatch> &matches)
{
    cv::Mat descriptors_1, descriptors_2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // Step1: detect Oriented FAST
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // Step2: extract BRIEF descriptor
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // Step3: match descriptors with hamming distance
    std::vector<cv::DMatch> raw_match;
    matcher->match(descriptors_1, descriptors_2, raw_match);

    // Step4: filter matches features
    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < raw_match.size(); ++i) {
        double dist = raw_match[i].distance;
        min_dist = std::min(min_dist, dist);
        max_dist = std::max(max_dist, dist);
    }

    std::cout << "-- Max dist: " << max_dist << std::endl;
    std::cout << "-- Min dist: " << min_dist << std::endl;

    for (int i = 0; i < raw_match.size(); ++i) {
        if (raw_match[i].distance < std::max(2*min_dist, 30.0)) {
            matches.push_back(raw_match[i]);
        }
    }
}

// Transform pixel coordinate to [X/Z, Y/Z, 1]
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat K) {
    return cv::Point2d((p.x - K.at<double>(0, 2))/K.at<double>(0, 0), (p.y - K.at<double>(1,2))/K.at<double>(1,1));
}

// Customize G2O Edge
/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(std::istream &in) override {}

  virtual bool write(std::ostream &out) const override {}
};

class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d& point): _point(point) {}

    bool read (std::istream &in) {}
    bool write (std::ostream &out) const {}
    virtual void computeError() {
        const VertexPose* pose = static_cast<VertexPose*> (_vertices[0]);
        _error = _measurement - pose->estimate() * _point;
    }

protected:
    Eigen::Vector3d _point;
};

// Bundle Adjustment
void bundleAdjustment(const std::vector<cv::Point3f> &pts1, const std::vector<cv::Point3f> &pts2,
                      cv::Mat &R, cv::Mat &t) {
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3>>  BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    g2o::OptimizationAlgorithm* solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //add vertex
    VertexPose* pose = new VertexPose();
    pose->setId(0);
    pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(pose);

    //add edges
    for (size_t i = 0; i < pts1.size(); i++) {
        EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
        edge->setVertex(0, pose);
        edge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    std::cout << "T = \n" << pose->estimate().matrix() << std::endl;

    Eigen::Matrix3d R_ = pose->estimate().rotationMatrix();
    Eigen::Vector3d t_ = pose->estimate().translation();
    R = (cv::Mat_<double>(3, 3) <<
        R_(0, 0), R_(0, 1), R_(0, 2),
        R_(1, 0), R_(1, 1), R_(1, 2),
        R_(2, 0), R_(2, 1), R_(2, 2)
    );
    t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));

}

int main(int argc, char *argv[]) {
    if (argc != 5) {
        std::cerr << "usage: pose_estimation_3d3d img1 img2 depth1 depth2" << std::endl;
        return 1;
    }

    cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;

    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "Find " << matches.size() << " pairs of matched keypoints" << std::endl;

    cv::Mat depth_1 = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat depth_2 = cv::imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat K = (cv::Mat_<double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point3f> pts1, pts2;

    for (cv::DMatch m:matches) {
        u_int16_t d1 = depth_1.at<u_int16_t>(keypoints_1[m.queryIdx].pt.y, keypoints_1[m.queryIdx].pt.x);
        u_int16_t d2 = depth_2.at<u_int16_t>(keypoints_2[m.trainIdx].pt.y, keypoints_2[m.trainIdx].pt.x);
        // std::cout << d1 << " " << d2 << std::endl;
        if (d1 == 0 || d2  == 0) {
            continue;
        }
        cv::Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        cv::Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;
        pts1.push_back(cv::Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(cv::Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    std::cout << "3d-3d pairs: " << pts1.size() << std::endl;
    cv::Mat R, t;
    bundleAdjustment(pts1, pts2, R, t);
    for (int i=0; i<5; ++i) {
        std::cout << "p1 = " << pts1[i] << std::endl;
        std::cout << "p2 = " << pts2[i] << std::endl;
        std::cout << "Transformation of p2 = " << R * (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t << std::endl;
    }
}
