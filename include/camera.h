/*******************************************************
 * Copyright (C) 2019, lingy17@mails.tsinghua.edu.cn
 * 
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#pragma once

#include <memory>
#include <Eigen/Core>

class Camera{

public:

    Camera();
    ~Camera();

    /**
     * @brief 2d 2 3d
     * 
     * @param p 
     * @param P 
     */
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;

    /**
     * @brief distort model
     * 
     * @param p_u 
     * @param d_u 
     */
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;

private:

    Eigen::Matrix3d             camera_matrix;
    Eigen::Matrix<double, 1, 5> distort_matrix;
    
    const int rows;
    const int cols;

    double fx;
    double fy;
    double cx;
    double cy;

    double invk_11;
    double invk_13;
    double invk_22;
    double invk_23;

    double k1;
    double k2;
    double p1;
    double p2;
    double k3;

    bool nodistort;
};