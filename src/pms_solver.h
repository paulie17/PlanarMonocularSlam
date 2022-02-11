#pragma once
#include <cmath>
#include "defs.h"
#include "camera.h"
#include "dataset_utils.h"

namespace pr{

    // planar monocular slam solver
    class pms_solver{
        public:        
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            pms_solver();

            void init(  Vector3fVector& odom_traj,
                        Vector3fVector& landmarks,
                        const MeasVector& measurements);
    
            //inline float kernelThreshold() const {return _kernel_thereshold;}

            //inline void setKernelThreshold(float kernel_threshold) 
            //{_kernel_thereshold=kernel_threshold;}

            void one_round();

        protected:
        
            void errorAndJacobian_odom( int pose_i_idx, 
                                        int pose_j_idx, 
                                        Matrix2_3f& J_odom_pose_i,
                                        Matrix2_3f& J_odom_pose_j,
                                        Eigen::Vector2f& odom_error);

            bool errorAndJacobian_proj( int pose_idx, 
                                        int landmark_idx,                                        
                                        Matrix2_3f& J_proj_landmark,
                                        Matrix2_3f& J_proj_pose,
                                        Eigen::Vector2f& proj_error);

            void boxplus();

            //float _kernel_thereshold;           //< threshold for the kernel
            Camera _camera;                     //< this variable holds the camera parameters
            Vector3fVector _odometry_displacements; //< store all the measurements of the odometer as displacements between the positions 
            
            const MeasVector* _measurements;
            Vector3fVector* _odom_traj_estimate;
            Vector3fVector* _landmarks_estimate;

            Eigen::Isometry3f _camera_to_robot;
            Eigen::MatrixXf _H;
            Eigen::VectorXf _b;
            Eigen::VectorXf _delta_x;
            
            float _chi_odom;
            float _chi_proj;

            int _n_of_landmarks;
    };
}