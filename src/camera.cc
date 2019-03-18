/*******************************************************
 * Copyright (C) 2019, lingy17@mails.tsinghua.edu.cn
 * 
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#include "camera.h"
#include "parameter.h"

Camera::Camera()
    : rows(IMAGE_ROWS), cols(IMAGE_COLS), nodistort(true)
{

    camera_matrix   = CAMERA_MATRIX;
    distort_matrix  = CAMERA_DISTORT;
    
    fx = camera_matrix(0, 0);
    fy = camera_matrix(0, 2);
    cx = camera_matrix(1, 1);
    cy = camera_matrix(1, 2);

    invk_11 = 1 / fx;
    invk_13 = -invk_11 * cx;
    invk_22 = 1 / fy;
    invk_23 = -invk_22 * cy;

    k1 = CAMERA_DISTORT(0, 0);
    
    if(k1 != 0){
        nodistort = false;

        k2 = CAMERA_DISTORT(0, 1);
        p1 = CAMERA_DISTORT(0, 2);
        p2 = CAMERA_DISTORT(0, 3);
        k3 = CAMERA_DISTORT(0, 4);
    }
    

}

Camera::~Camera(){
    
}

void Camera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const {

    double x2_u = p_u(0) * p_u(0);
    double y2_u = p_u(1) * p_u(1);

    double xy_u = p_u(0) * p_u(1);

    double rho2_u = x2_u + y2_u;

    double rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * xy_u + p2 * (rho2_u + 2.0 * x2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * xy_u + p1 * (rho2_u + 2.0 * y2_u);

}

void Camera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const {

    double x_d = invk_11 * p(0) + invk_13;
    double y_d = invk_22 * p(1) + invk_23;

    if(nodistort){
        P = Eigen::Vector3d(x_d, y_d, 1.0);
        return;
    }
    else{

        
        Eigen::Vector2d d_u;

        // 循环进行畸变矫正
        distortion(Eigen::Vector2d(x_d, y_d), d_u);

        double x_u = x_d - d_u(0);
        double y_u = y_d - d_u(1);

        for(int i = 1; i < 8; ++i)
        {
            distortion(Eigen::Vector2d(x_u, y_u), d_u);
            x_u = x_d - d_u(0);
            y_u = y_d - d_u(1);
        }

        P = Eigen::Vector3d(x_u, y_u, 1.0);
        return;
    }

}