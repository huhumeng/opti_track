/*******************************************************
 * Copyright (C) 2019, lingy17@mails.tsinghua.edu.cn
 * 
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#pragma once

#include <string>
#include <Eigen/Core>

extern std::string DATA_ROOT_DIR;
extern int DATA_SEQUENCE_NUM;

extern int FEATURE_NUM;
extern int MIN_DISTANCE;
extern int BORDER_SIZE;

extern int NUM_OF_CAM;

extern int IMAGE_ROWS;
extern int IMAGE_COLS;

extern Eigen::Matrix3d CAMERA_MATRIX;
extern Eigen::Matrix<double, 1, 5> CAMERA_DISTORT;

void readAllParameters();