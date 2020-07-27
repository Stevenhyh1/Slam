/****************************
 * 题目：SLAM问题的目标之一就是精确的估计相机运动的轨迹（姿态），如果我们将相机运动的轨迹绘制出来，就可以直观的观察它的运动是否符合预期。
 * 给定一个轨迹文件trajectory.txt，该文件的每一行由若干个数据组成，格式为 [time, tx, ty, tz, qx, qy, qz, qw],
 * 其中 time 为时间，tx,ty,tz 为平移部分，qx,qy,qz,qw 是四元数表示的旋转部分，请完成数据读取部分的代码，绘制部分代码已经给出。
 *
* 本程序学习目标：
 * 熟悉李代数库Sophus安装及基本操作
 * 熟悉数据流读取基本操作
 * 需要安装pangolin，注意异常数据处理
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.10
****************************/

#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

using namespace std;

// path to trajectory file
string trajectory_file = "../trajectory.txt";
bool LoadTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &poses);
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses);

int main(int argc, char **argv) {

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses;

    /// implement pose reading code
    // 开始你的代码
    if (!LoadTrajectory(poses)) {
        return -1;
    }

    // 结束你的代码

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

bool LoadTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &poses) {
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Sophus::SE3d T;
    

    fstream trajectoryfile;
    trajectoryfile.open(trajectory_file, std::ios::in);
    if (!trajectoryfile.is_open()) {
        std::cerr << "The file doesn't exist!" << std::endl;
        return false;
    } else {
        int lineidx=0;
        std::string linetxt, timestamp;
        while (std::getline(trajectoryfile, linetxt) && !linetxt.empty()) {
            std::istringstream strData(linetxt);
            double t0, t1, t2, qx, qy, qz, qw;
            strData >> timestamp >> t0 >> t1 >> t2 >> qx >> qy >> qz >> qw;
            lineidx++;
            if (timestamp == "#" || timestamp == "-1.#INF") {
                std::cout << lineidx << timestamp << std::endl;
                continue;
            }
            t = Eigen::Vector3d(t0, t1, t2);
            q = Eigen::Quaterniond(qw, qx, qy, qz);
            T = Sophus::SE3d(q, t);
            poses.push_back(T);
        }
    }
    trajectoryfile.close();
    return true;
}

// 无需改动以下绘图程序
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   
    }

}