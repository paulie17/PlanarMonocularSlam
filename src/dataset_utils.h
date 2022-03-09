#pragma once
#include "defs.h"
#include "camera.h"
#include <fstream>
#include <iomanip>

namespace pms{

    struct Measurement{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Camera current_camera_position;
    
        int seq;

        IntVector detected_landmarks;
        Vector10fVector appearances;
        Vector2fVector landmarks_img_pts;
        Vector3fVector bearings;

        Eigen::Vector3f odom_pose;
        Eigen::Vector3f gt_pose;
    };

    typedef std::vector<Measurement, Eigen::aligned_allocator<Measurement> > MeasVector;

    void load_trajectory(Vector3fVector& odom_trajectory, Vector3fVector& gt_trajectory);

    void load_camera_data(  int& z_near,int& z_far,int& width, int& height,
                            Eigen::Matrix3f& camera_matrix, Eigen::Isometry3f& camera_to_robot);

    Camera load_camera_data();

    
    void load_measurements(MeasVector& measurements);
    
    Vector3fVector initial_guess(MeasVector& measurements,int NUM_LANDMARKS,IntVector& discarded);

    template <class T>
    void prepare_for_plotting(const T& points,FloatVector& x, FloatVector& y, FloatVector& z){
        x.clear();
        y.clear();
        z.clear();
        int num_points = points.size();
        for (int i = 0; i < num_points; i++){
            x.push_back(points[i](0));
            y.push_back(points[i](1));
            z.push_back(points[i](2));
        }
    }    
    template <class T>
    void prepare_for_plotting(const T& points,FloatVector& x, FloatVector& y){
        x.clear();
        y.clear();
        int num_points = points.size();
        for (int i = 0; i < num_points; i++){
            x.push_back(points[i](0));
            y.push_back(points[i](1));
        }
    }    
    
    void match_features(const Vector10fVector& desc1,
                    const Vector10fVector& desc2,
                    const float ratio,
                    IntPairVector& matches);
    //return the number of landmarks(highest id)
    int data_association(MeasVector& measurements);

    Vector3fVector load_landmarks_gt();

    const int NUM_MEASUREMENTS = 172;

}