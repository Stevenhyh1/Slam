/****************************
 * 题目：现有一个运动着的相机拍摄的连续两张图片，其中特征点匹配部分已经完成。
 * 请根据两帧图像对应的匹配点计算基础矩阵，并利用该矩阵绘制出前10个特征点对应的极线。
 *
* 本程序学习目标：
 * 理解掌握对极约束的原理
 * 熟悉OpenCV编程
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.11
****************************/
#include<iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;
int main( int argc, char** argv )
{

    Mat rgb1 = imread( "../rgb1.ppm");
    Mat rgb2 = imread( "../rgb2.ppm");

    // Ptr<FeatureDetector> detector;
    // Ptr<DescriptorExtractor> descriptor;
    
    // detector = FeatureDetector::create("ORB");
    // descriptor = DescriptorExtractor::create("ORB");

    // vector< KeyPoint > kp1, kp2;
    // detector->detect( rgb1, kp1 );
    // detector->detect( rgb2, kp2 );

    // // 计算描述子
    // Mat desp1, desp2;
    // descriptor->compute( rgb1, kp1, desp1 );
    // descriptor->compute( rgb2, kp2, desp2 );

    Ptr<ORB> detector = ORB::create();
    std::vector<KeyPoint> kp1, kp2;
    Mat desp1, desp2;
    detector->detectAndCompute( rgb1, noArray(), kp1, desp1 );
    detector->detectAndCompute( rgb2, noArray(), kp2, desp2 );

    // 匹配描述子
    vector< DMatch > matches;
    BFMatcher matcher;
    matcher.match( desp1, desp2, matches );
    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    // 筛选匹配对
    vector< DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }


    vector< Point2f > pts1, pts2;
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        pts1.push_back(kp1[goodMatches[i].queryIdx].pt);
        pts2.push_back(kp2[goodMatches[i].trainIdx].pt);
    }

    cout << "Find total " << pts1.size() << " matches after filtering" << endl;
    Mat img_matches;
    drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow("Good Matches", img_matches );
    // 请先计算基础矩阵并据此绘制出前10个匹配点对应的对极线，可以调用opencv函数
    // ----------- 开始你的代码 --------------//
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(pts1, pts2, CV_FM_8POINT);
    cout << "Fundamental matrix is \n" << fundamental_matrix << endl;
    vector<Vec<float,3>> epilines1, epilines2;
    // Output vector of the epipolar lines corresponding to the points in the other image. Each line ax+by+c=0 is encoded by 3 numbers (a,b,c)
    // y = (-c - a*x)/b
    computeCorrespondEpilines(pts1, 1, fundamental_matrix, epilines1);
    computeCorrespondEpilines(pts2, 2, fundamental_matrix, epilines2);

    RNG rng;
    for (size_t i=0; i<10; ++i){
        
        Scalar color = Scalar(rng(256), rng(256), rng(256));
        circle(rgb1, pts1[i], 3, color);
        float x1 = 0, y1 = -epilines2[i][2] / epilines2[i][1];
        float x2 = rgb1.cols, y2 = (-epilines2[i][2] - epilines2[i][0] * x2) / epilines2[i][1];
        line(rgb1, Point(x1, y1), Point(x2, y2), color);

        circle(rgb2, pts2[i], 3, color);
        float x3 = 0, y3 = -epilines1[i][2] / epilines1[i][1];
        float x4 = rgb2.cols, y4 = (-epilines1[i][2] - epilines1[i][0] * x4) / epilines1[i][1];
        line(rgb2, Point(x3, y3), Point(x4, y4), color);
    
    }
    // ----------- 结束你的代码 --------------//
    imshow("epiline1", rgb2);
    imshow("epiline2", rgb1);
    waitKey(0);
    return 0;
}
