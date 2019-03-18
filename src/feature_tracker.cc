/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#include "feature_tracker.h"

#include "parameter.h"
#include "slam_util.h"
#include "camera.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>


FeatureTracker::FeatureTracker()
    : rows(IMAGE_ROWS), cols(IMAGE_COLS)
{
    for(int i=0; i<2; ++i)
        camera[i] = std::make_shared<Camera>();
    
    n_id = 0;

    hasPrediction = false;
    calcBackward  = true;
    debugPlot     = true;

    writer.open("./optical_flow.avi", CV_FOURCC('M', 'J', 'P', 'G'), 20, cv::Size(1241, 376), true);
}

void FeatureTracker::setMask(){

    mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));

    // 前一个int为追踪的次数 后一个int为id
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

    for(unsigned int i=0; i<cur_pts.size(); ++i){
        cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(cur_pts[i], ids[i])));
    }

    // 按照追踪次数排序
    std::sort(cnt_pts_id.begin(), cnt_pts_id.end(),
        [](const std::pair<int, std::pair<cv::Point2f, int>>& a, 
           const std::pair<int, std::pair<cv::Point2f, int>>& b){
               return a.first > b.first;
           }
    
    );


    // 按优先级过滤特征点
    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for(auto& it : cnt_pts_id){

        if(mask.at<uchar>(it.second.first) == 255){
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

            cv::circle(mask, it.second.first, MIN_DISTANCE, 0, -1);            
        }
    }
}

void FeatureTracker::addPoints(){

    for(auto& p : n_pts){

        cur_pts.push_back(p);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
    }


}

FeatureTracker::FeatureFrame FeatureTracker::trackImage(double time_in, const cv::Mat& image0, const cv::Mat& image1){
    
    cur_time = time_in;

    cur_img = image0;
    cur_pts.clear();

    

    if(prev_pts.size() > 0){

        std::vector<uchar> status;
        std::vector<float> err;
        
        if(hasPrediction)
        {
            
            cur_pts = predict_pts;

            cv::calcOpticalFlowPyrLK(
                prev_img, cur_img, prev_pts, cur_pts, status, err, 
                cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), 
                cv::OPTFLOW_USE_INITIAL_FLOW
            );

            int succ_num = 0;
            for(auto& t : status)
                if(t)
                    succ_num++;
            

            if(succ_num < 10)
                cv::calcOpticalFlowPyrLK(
                    prev_img, cur_img, prev_pts, cur_pts, status, err, 
                    cv::Size(21, 21), 3
                );
        }else{
            cv::calcOpticalFlowPyrLK(
                prev_img, cur_img, prev_pts, cur_pts, status, err, 
                cv::Size(21, 21), 3
            );
        }

        // 通过反向光流往回计算，提高精度
        if(calcBackward){
            std::vector<uchar> reverse_status;
            std::vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(
                cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, 
                cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), 
                cv::OPTFLOW_USE_INITIAL_FLOW
            );

            for(size_t i=0; i<status.size(); ++i){
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5){
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        
        for(size_t i=0; i<cur_pts.size(); ++i){
            if(status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        }

        reducevecotr(prev_pts, status);
        reducevecotr(cur_pts, status);
        reducevecotr(ids, status);
        reducevecotr(track_cnt, status);

    }
    
    for(auto& n : track_cnt)
        n++;
        
    // 初始化
    setMask();

    int n_max_cnt = FEATURE_NUM - static_cast<int>(cur_pts.size());

    if(n_max_cnt > 0)
    {
        if(mask.empty())
            Warn("mask is empty\n");
        if (mask.type() != CV_8UC1)
            Warn("mask type error\n");

//        if(mask.empty())
//            cv::goodFeaturesToTrack(cur_img, n_pts, n_max_cnt, 0.01, MIN_DISTANCE);
        cv::goodFeaturesToTrack(cur_img, n_pts, n_max_cnt, 0.01, MIN_DISTANCE, mask);
    }
    else
        n_pts.clear();

    addPoints();

    undistortedPts(cur_pts, cur_un_pts, camera[0]);
    ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map, pts_velocity);

    if(!image1.empty())
    {
        std::vector<cv::Point2f> reverseLeftPts;
        std::vector<uchar> status, statusRightLeft;
        std::vector<float> err;

        // cur left ---- cur right
        cv::calcOpticalFlowPyrLK(
            cur_img, image1, cur_pts, cur_right_pts, status, err, 
            cv::Size(21, 21), 3
        );

        if(calcBackward){

            // cur right ---- cur left
            cv::calcOpticalFlowPyrLK(
                image1, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, 
                cv::Size(21, 21), 3
            );

            for(size_t i = 0; i < status.size(); ++i)
            {
                if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                    status[i] = 1;
                else
                    status[i] = 0;
            }
        }

        ids_right = ids;
        reducevecotr(cur_right_pts, status);
        reducevecotr(ids_right, status);

        undistortedPts(cur_right_pts, cur_un_right_pts, camera[1]);
        ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map, right_pts_velocity);

        prev_un_right_pts_map = cur_un_right_pts_map;
    }

    if(debugPlot)
        drawTrack(cur_img, ids, cur_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); ++i)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    

    FeatureFrame featureFrame;

    for(size_t i = 0; i < ids.size(); ++i)
    {

        int feature_id = ids[i];
        double x, y, z;

        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;

        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if(!image1.empty())
    {
        for(size_t i = 0; i < ids_right.size(); ++i)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;

            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }



    return featureFrame;
}

inline bool FeatureTracker::inBorder(const cv::Point2f& pt) const 
{
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);


    return BORDER_SIZE <= img_x &&
           img_x <= cols - BORDER_SIZE &&
           BORDER_SIZE <= img_y &&
           img_y <= rows - BORDER_SIZE;
}

inline double FeatureTracker::distance(const cv::Point2f& pt1, const cv::Point2f& pt2) const{

//    printf("%lf %lf\n", pt1.x, pt1.y);

    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;

    return sqrt(dx * dx + dy * dy);

}


template <typename T>
void FeatureTracker::reducevecotr(std::vector<T>& v, const std::vector<uchar>& status){

    size_t j = 0;
    for(size_t i=0; i<v.size(); ++i){
        if(status[i])
            v[j++] = v[i];
    }

    v.resize(j);

}

void FeatureTracker::undistortedPts(const std::vector<cv::Point2f>& pts,
                                    std::vector<cv::Point2f>& un_pts,
                                    CameraPtr cam){

    un_pts.clear();

    for(const auto& pt : pts){

        Eigen::Vector2d a(pt.x, pt.y);
        Eigen::Vector3d b;



        cam->liftProjective(a, b);

        un_pts.push_back(cv::Point2f(b(0), b(1)));
    }

}

void FeatureTracker::ptsVelocity(std::vector<int>& ids, 
                                 std::vector<cv::Point2f>& pts,
                                 std::map<int, cv::Point2f>& cur_id_pts,
                                 std::map<int, cv::Point2f>& prev_id_pts,
                                 std::vector<cv::Point2f>& pts_velocity){
    
    pts_velocity.clear();

    cur_id_pts.clear();

    for(unsigned int i=0; i<ids.size(); ++i)
        cur_id_pts.insert(std::make_pair(ids[i], pts[i]));
    
    if(!prev_id_pts.empty()){
        double dt = cur_time - prev_time;

        for(unsigned int i=0; i<pts.size(); ++i){
            
            std::map<int, cv::Point2f>::iterator it;

            it = prev_id_pts.find(ids[i]);

            if(it != prev_id_pts.end()){

                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;

                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    else
    {
        for(unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }


}

void FeatureTracker::drawTrack(const cv::Mat& imLeft, 
                               std::vector<int>& curLeftIds,
                               std::vector<cv::Point2f>& curLeftPts,
                               std::map<int, cv::Point2f>& prevLeftPtsMap){

    imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for(size_t j = 0; j < curLeftPts.size(); ++j)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }

    // if(!imRight.empty()){
    //     for(size_t i = 0; i < curRightPts.size(); ++i)
    //     {
    //         cv::Point2f rightPt = curRightPts[i];
    //         rightPt.x += cols;
    //         cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
    //     }
    // }

    std::map<int, cv::Point2f>::iterator mapIt;
    
    for(size_t i = 0; i < curLeftIds.size(); ++i)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);

        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    cv::imshow("tracking", imTrack);
    cv::waitKey(1);

    writer << imTrack;
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}
