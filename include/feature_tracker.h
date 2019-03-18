/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#pragma once

#include <map>
#include <vector>
#include <memory>


#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>

class Camera;

class FeatureTracker{
    typedef std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureFrame;


public:

    FeatureTracker();

    FeatureFrame trackImage(double time_in, const cv::Mat& image0, const cv::Mat& image1 = cv::Mat());

private:

    cv::VideoWriter writer;
    
    typedef std::shared_ptr<Camera> CameraPtr;

    bool inBorder(const cv::Point2f& pt) const;
    double distance(const cv::Point2f& pt1,
                    const cv::Point2f& pt2) const;

    void addPoints();
    void setMask();

    template <typename T>
    void reducevecotr(std::vector<T>& v, const std::vector<uchar>& status);
    
    void undistortedPts(const std::vector<cv::Point2f>&,
                        std::vector<cv::Point2f>&,
                        CameraPtr);

    void ptsVelocity(std::vector<int>& ids, 
                     std::vector<cv::Point2f>& pts,
                     std::map<int, cv::Point2f>& cur_id_pts,
                     std::map<int, cv::Point2f>& prev_id_pts,
                     std::vector<cv::Point2f>&);
    
    void drawTrack(const cv::Mat& imLeft,
                   std::vector<int>& curLeftIds,
                   std::vector<cv::Point2f>& curLeftPts,
                   std::map<int, cv::Point2f>& prevLeftPtsMap);

    cv::Mat getTrackImage();

    union {
        const int rows;
        const int height;
    };
    
    union {
        const int cols;
        const int width;
    };
    

    CameraPtr camera[2];

    // 存储特征点
    std::vector<cv::Point2f> n_pts;
    std::vector<cv::Point2f> predict_pts; // 用于光流追踪猜测
    
    std::vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    std::vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;

    std::map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    std::map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    std::map<int, cv::Point2f> prevLeftPtsMap;
    
    // 光流速度
    std::vector<cv::Point2f> pts_velocity, right_pts_velocity;

    // 存储id与追踪次数
    int                 n_id;           // 特征点id
    std::vector<int>    ids, ids_right;
    std::vector<int>    track_cnt;
    
    // 存储图像
    cv::Mat prev_img, cur_img;

    // mask
    cv::Mat mask, fisheye_mask;

    // debug show
    cv::Mat imTrack;

    // 特征点追踪配置
    bool hasPrediction; // 用于设置是否有光流的预测
    bool calcBackward;  // 设置是否计算反向光流
    bool debugPlot;

    double cur_time, prev_time;
};