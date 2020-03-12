#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace std;

void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    vector<cv::KeyPoint> &keypoints_1,
    vector<cv::KeyPoint> &keypoints_2,
    vector<cv::DMatch> &matches
);

void pose_estimation_2d2d(
    vector<cv::KeyPoint> keypoints_1,
    vector<cv::KeyPoint> keypoints_2,
    vector<cv::DMatch> matches,
    cv::Mat &R, cv::Mat &t
);

cv::Point2d pixel2cam( const cv::Point2d &p, const cv::Mat &K);

int main(int argc, char *argv[]) {
    if (argc !=3 ) {
        cerr << "usage: pose_estimation_2d2d img_1 img_2" << endl;
        return 1;
    }

    cv::Mat img_1 = cv::imread(argv[1],CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2],CV_LOAD_IMAGE_COLOR);
    assert (img_1.data != nullptr && img_2.data != nullptr);

    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);

    cv::Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
    

    cv::Mat t_x =
    (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
      t.at<double>(2, 0), 0, -t.at<double>(0, 0),
      -t.at<double>(1, 0), t.at<double>(0, 0), 0);

    cout << "t^R=" << endl << t_x * R << endl;

  //-- 验证对极约束
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (cv::DMatch m: matches) {
    cv::Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    cv::Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
    cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    cv::Mat d = y2.t() * t_x * R * y1;
    cout << "epipolar constraint = " << d << endl;
    }
    return 0;

}

void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    vector<cv::KeyPoint> &keypoints_1,
    vector<cv::KeyPoint> &keypoints_2,
    vector<cv::DMatch> &matches) 
    {
    cv::Mat descriptor_1, descriptor_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    descriptor->compute(img_1, keypoints_1, descriptor_1);
    descriptor->compute(img_2, keypoints_2, descriptor_2);

    vector<cv::DMatch> match;
    matcher->match(descriptor_1, descriptor_2, match);

    double min_dist = 10000, max_dist = 0;

    for (int i=0; i<descriptor_1.rows; i++) {
        double disti = match[i].distance;
        min_dist = min(disti, min_dist);
        max_dist = max(disti, max_dist);
    }

    printf("--Max dist: %f \n", max_dist);
    printf("--Min dist: %f \n", min_dist);

    for (int i=0; i<descriptor_1.rows; i++) {
        if (match[i].distance < max(2*min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
    }

    cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
        return cv::Point2d(
            (p.x - K.at<double>(0,2))/K.at<double>(0,0),
            (p.y - K.at<double>(1,2))/K.at<double>(1,1)
        );
    }

    void pose_estimation_2d2d(vector<cv::KeyPoint> keypoints_1,
                              vector<cv::KeyPoint> keypoints_2,
                              vector<cv::DMatch> matches,
                              cv::Mat &R, cv::Mat &t) {
        
        cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

        vector<cv::Point2d> points1;
        vector<cv::Point2d> points2;

        for (int i=0; i<(int)matches.size(); i++) {
            points1.push_back(keypoints_1[matches[i].queryIdx].pt);
            points2.push_back(keypoints_2[matches[i].queryIdx].pt);
        }

        cv::Mat fundamental_matrix;
        fundamental_matrix = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);

        cv::Mat essential_matrix;
        cv::Point2d principal_point(325.1, 249.7);
        double focal = 521;
        essential_matrix = cv::findEssentialMat(points1, points2, focal, principal_point);

        cv::Mat homography_matrix;
        homography_matrix = cv::findHomography(points1, points2, CV_RANSAC, 3);

        cv::recoverPose(essential_matrix, points1, points2, R, t, focal, principal_point);

        }
    