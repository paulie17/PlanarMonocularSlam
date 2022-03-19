#pragma once
#include "defs.h"
#include "camera.h"
#include <fstream>
#include <iomanip>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/irange.hpp>


namespace pms{

    struct Measurement{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Camera current_camera_position;
    
        int seq;

        IntVector detected_landmarks;
        Vector2dVector landmarks_img_pts;
        Vector3dVector bearings;

        Eigen::Vector3d odom_pose;
        Eigen::Vector3d gt_pose;
    };

    typedef std::vector<Measurement, Eigen::aligned_allocator<Measurement> > MeasVector;

    void load_trajectory(Vector3dVector& odom_trajectory, Vector3dVector& gt_trajectory);

    void load_camera_data(  double& z_near,double& z_far,int& width, int& height,
                            Eigen::Matrix3d& camera_matrix, Eigen::Isometry3d& camera_to_robot);

    Camera load_camera_data();

    //return the number of landmarks(highest id)
    int load_measurements(MeasVector& measurements);
    
    Vector3dVector initial_guess(MeasVector& measurements,int NUM_LANDMARKS,IntVector& discarded);

    template <class T>
    void prepare_for_plotting(const T& points,DoubleVector& x, DoubleVector& y, DoubleVector& z){
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
    void prepare_for_plotting(const T& points,DoubleVector& x, DoubleVector& y){
        x.clear();
        y.clear();
        int num_points = points.size();
        for (int i = 0; i < num_points; i++){
            x.push_back(points[i](0));
            y.push_back(points[i](1));
        }
    }    
    

    Vector3dVector load_landmarks_gt();

    void make_map(IntVector& discarded, int NUM_LANDMARKS, std::map<int,int>& index_to_id_Map);

    const int NUM_MEASUREMENTS = 200;

}