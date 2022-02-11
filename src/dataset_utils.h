#pragma once
#include "defs.h"
#include "camera.h"
#include <fstream>
#include <iomanip>

namespace pr{

    struct Measurement{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Camera current_camera_position;
    
        int seq;

        IntVector detected_landmarks;
        Vector2fVector landmarks_img_pts;
        Vector3fVector bearings;

        Eigen::Vector3f odom_pose;
        Eigen::Vector3f gt_pose;
    };

    typedef std::vector<Measurement, Eigen::aligned_allocator<Measurement> > MeasVector;

    Vector3fVector load_trajectory();

    void load_camera_data(  int& z_near,int& z_far,int& width, int& height,
                            Eigen::Matrix3f& camera_matrix, Eigen::Isometry3f& camera_to_robot);

    Camera load_camera_data();

    //return the number of landmarks(higher id)
    int load_measurements(MeasVector& measurements);

    Vector3fVector load_landmarks_gt();

    const int NUM_MEASUREMENTS = 200;
    //const int NUM_LANDMARKS= 1000;

}