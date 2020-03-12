#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cerr << "usages: feature extraction img1 img2" << endl;
        return 1;
    }

    cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    //Initialize
    vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    //Step 1： Oriented FAST
    detector->detect(img_1, keypoints1);
    detector->detect(img_2, keypoints2);

    //Step 2: Calculate BRIEF descriptor
    descriptor->compute(img_1, keypoints1, descriptors_1);
    descriptor->compute(img_2, keypoints2, descriptors_2);

    cv::Mat outimg1;
    cv::drawKeypoints(img_1, keypoints1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB features", outimg1);
    
    //Step 3: Matching
    vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);

    //Step 4: Screening
    auto min_max  = minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2){return m1.distance < m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    cout << "Min distance: " << min_dist << endl;
    cout << "Max distance: " << max_dist << endl;

    std::vector<cv::DMatch> good_matches; 
    for (int i=0; i<descriptors_1.rows; i++) {
        if (matches[i].distance <= max(2*min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    //Plotting
    cv::Mat img_match; 
    cv::Mat img_goodmatch;
    cv::drawMatches(img_1, keypoints1, img_2, keypoints2, matches, img_match);
    cv::drawMatches(img_1, keypoints1, img_2, keypoints2, good_matches, img_goodmatch);
    cv::imshow("all matches", img_match);
    cv::imshow("good matches", img_goodmatch);
    cv::waitKey(0);

    return 0;
    

}