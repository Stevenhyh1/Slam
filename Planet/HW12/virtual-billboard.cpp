/****************************
 * 实现虚拟广告牌的效果。
 * 提供两张图，一张是“计算机视觉life”公众号的logo，另外一张是带广告牌的原图，请用单应矩阵实现将原图中广告牌替换为提供的logo的效果。
 * 利用OpenCV函数，通过鼠标点击来选择要替换的广告牌的四个顶点。
 *
* 本程序学习目标：
 * 理解掌握单应矩阵的使用
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.11
****************************/

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

struct userdata{
    Mat im;
    vector<Point2f> points;
};


void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        userdata *data = ((userdata *) data_ptr);
        circle(data->im, Point(x,y),3,Scalar(0,255,255), 5, CV_AA);
        imshow("Image", data->im);
        if (data->points.size() < 4)
        {
            data->points.push_back(Point2f(x,y));
        }
    }
    
}



int main( int argc, char** argv)
{

    // Read in the image.
    Mat im_src = imread("../cvlife.jpg");
    Size size = im_src.size();
   
    // Create a vector of points.
    vector<Point2f> pts_src;
    pts_src.push_back(Point2f(0,0)); // left top
    pts_src.push_back(Point2f(size.width - 1, 0)); // right top
    pts_src.push_back(Point2f(size.width - 1, size.height -1)); // right bottom
    pts_src.push_back(Point2f(0, size.height - 1 )); // left bottom
    
    // Destination image
    Mat im_dst = imread("../ad.jpg");

    // Set data for mouse handler
    Mat im_temp = im_dst.clone();
    userdata data;
    data.im = im_temp;


    //show the image
    imshow("Image", im_temp);
    
    cout << "Click on four corners of a billboard and then press ENTER" << endl;
    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    waitKey(0);
    
    // ----------  开始你的代码  --------------
    vector<Point2f> pts_dest = data.points;
    Mat H = findHomography(pts_src, pts_dest);
    cout << "Homography matrix = \n" << H << endl;
    Point pts_dst[4];
    for (size_t i=0; i<4; ++i){
        pts_dst[i] = data.points[i];
    }
    warpPerspective(im_src, im_temp, H, im_temp.size());
    fillConvexPoly(im_dst, pts_dst, 4, Scalar(0));
    im_dst = im_dst + im_temp;

    // ----------  结束你的代码  --------------
    // Display image.
    imshow("Image", im_dst);
    waitKey(0);

    return 0;
}
