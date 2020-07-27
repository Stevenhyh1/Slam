#include "slamBase.hpp"

PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera)
{
    PointCloud::Ptr cloud ( new PointCloud );
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}


PointCloud::Ptr pointCloudFusion( PointCloud::Ptr &original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera )
{
	// ---------- 开始你的代码  ------------- -//
	// 简单的点云叠加融合
    PointCloud::Ptr newCloud(new PointCloud());
    newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);
    pcl::transformPointCloud(*newCloud, *newCloud, T.matrix());
    *original += *newCloud;
    
    return original;
	// ---------- 结束你的代码  ------------- -//
}


void readCameraTrajectory(string camTransFile, vector<Eigen::Isometry3d> &poses)
{
    ifstream fcamTrans(camTransFile);
    if(!fcamTrans.is_open())
    {
        cerr << "trajectory is empty!" << endl;
        return;
    }

   	// ---------- 开始你的代码  ------------- -//
	// 参考作业8 绘制轨迹
    string linetxt;
    while (getline(fcamTrans, linetxt) && !linetxt.empty()) {
        if (linetxt[0] == '#') {
            continue;
        }
        istringstream posedata(linetxt);
        double tx, ty, tz, qx, qy, qz, qw;
        posedata >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translate(Eigen::Vector3d(tx, ty, tz));
        pose.rotate(Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix());
        poses.push_back(pose);
        // cout << pose.rotation() << endl;
        // cout << pose.translation() << endl;
    }
	// ---------- 结束你的代码  ------------- -//
}