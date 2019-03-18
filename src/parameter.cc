/*******************************************************
 * Copyright (C) 2019, lingy17@mails.tsinghua.edu.cn
 * 
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/



#include "parameter.h"
#include "config.h"

#include <time.h>

#include <fstream>
#include <iomanip>


std::string DATA_ROOT_DIR;
int DATA_SEQUENCE_NUM;

int FEATURE_NUM;
int MIN_DISTANCE;
int BORDER_SIZE;

int NUM_OF_CAM;

int IMAGE_ROWS;
int IMAGE_COLS;

Eigen::Matrix3d CAMERA_MATRIX;
Eigen::Matrix<double, 1, 5> CAMERA_DISTORT;

void readAllParameters(){

    DATA_ROOT_DIR = Config::getParam<std::string>("dataset.root_dir");
    DATA_SEQUENCE_NUM = Config::getParam<int>("dataset.sequence_num");

    std::stringstream ss;
    ss << std::setfill('0') << std::setw(2) << DATA_SEQUENCE_NUM;

    DATA_ROOT_DIR += ss.str();
    
    FEATURE_NUM = Config::getParam<int>("param.feature_num");
    MIN_DISTANCE = Config::getParam<int>("param.min_distance");
    BORDER_SIZE = Config::getParam<int>("param.border_size");
    
    NUM_OF_CAM = Config::getParam<int>("camera.camera_num");

    IMAGE_ROWS = Config::getParam<int>("camera.height");
    IMAGE_COLS = Config::getParam<int>("camera.width");

    double fx = Config::getParam<double>("camera.fx");
    double fy = Config::getParam<double>("camera.fy");
    double cx = Config::getParam<double>("camera.cx");
    double cy = Config::getParam<double>("camera.cy");

    CAMERA_MATRIX.setIdentity();
    CAMERA_MATRIX(0, 0) = fx;
    CAMERA_MATRIX(0, 2) = cx;
    CAMERA_MATRIX(1, 1) = fy;
    CAMERA_MATRIX(1, 2) = cy;

    CAMERA_DISTORT(0, 0) = Config::getParam<double>("camera.k1");
    CAMERA_DISTORT(0, 1) = Config::getParam<double>("camera.k2");
    CAMERA_DISTORT(0, 2) = Config::getParam<double>("camera.p1");
    CAMERA_DISTORT(0, 3) = Config::getParam<double>("camera.p2");
    CAMERA_DISTORT(0, 4) = Config::getParam<double>("camera.k3");

}
 