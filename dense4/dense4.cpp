#include <iostream>
#include <fstream>

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
int main(int argc, char **argv) {
    int N=79;
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d> poses;         // 相机位姿

    ifstream fin("../associations.txt");
    if (!fin) {
        cerr << "cannot find pose file" << endl;
        return 1;
    }

    for (int i = 0; i < N; i++) {
        string data[12] ;
        for (int i = 0; i < 12; i++) {
            fin >> data[i];
        }
        colorImgs.push_back(cv::imread("/home/wood/data_for_orbslam2/rgbd_dataset_freiburg1_xyz/"+data[1]));
        depthImgs.push_back(cv::imread("/home/wood/data_for_orbslam2/rgbd_dataset_freiburg1_xyz/"+data[3], -1)); // 使用-1读取原始图像
        double double_data[7];
        for(int j=5;j<12;j++)
            double_data[j-5]=atof(data[j].c_str());
        Eigen::Quaterniond q(double_data[6], double_data[3], double_data[4], double_data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(double_data[0], double_data[1], double_data[2]));
        poses.push_back(T);//位姿
    }
    /*
const double camera_factor=5000;
const double camera_cx=325.5;
const double camera_cy=253.5;
const double camera_fx=518.0;
const double camera_fy=519.0;
     */
    // 计算点云并拼接
    // 相机内参 
    double cx = 318.6;
    double cy = 255.3;
    double fx = 517.3;
    double fy = 516.5;
    double depthScale = 5000.0;

    cout << "正在将图像转换为点云..." << endl;

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    //viewer
    pcl::visualization::CloudViewer viewer ("Viewer");
    for (int i = 0; i < N; i++) {
        PointCloud::Ptr current(new PointCloud);
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;//相机坐标
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;//世界坐标

                PointT p;//the fisrt 3 dimensions of p is directions and the next 3 is rgb
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                current->points.push_back(p);
            }
        // depth filter and statistical removal 
        /*
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);
         */
        
        (*pointCloud) += *current;
        cv::imshow("img",colorImgs[i]);
        cv::waitKey(30);
        viewer.showCloud(pointCloud);
    }
    while (!viewer.wasStopped ())
    {
        
    }
/*
 1.pointCloud is the final pcl
 2.tmp->statistical_filter
 3.current(->statistical_filter)->tmp->pointCloud
 */
    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;

    // voxel filter 
    /*
    pcl::VoxelGrid<PointT> voxel_filter;
    double resolution = 0.03;
    voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
    PointCloud::Ptr tmp(new PointCloud);
    voxel_filter.setInputCloud(pointCloud);
    voxel_filter.filter(*tmp);
    tmp->swap(*pointCloud);
    cout << "滤波之后，点云共有" << pointCloud->size() << "个点." << endl;
     */


    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
    return 0;
}
