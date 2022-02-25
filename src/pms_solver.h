#pragma once
#include <cmath>
#include "defs.h"
#include "camera.h"
#include "dataset_utils.h"

namespace pms{

    // planar monocular slam solver
    class pms_solver{
        public:        
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            pms_solver();

            void init(  Vector3fVector& odom_traj,
                        Vector3fVector& landmarks,
                        const MeasVector& measurements);
    
            inline float ProjkernelThreshold() const {return _proj_kernel_threshold;}

            inline void setProjKernelThreshold(float kernel_threshold) 
            {_proj_kernel_threshold=kernel_threshold;}

            inline float OdomkernelThreshold() const {return _odom_kernel_threshold;}

            inline void setOdomKernelThreshold(float kernel_threshold) 
            {_odom_kernel_threshold=kernel_threshold;}

            //! chi square of the odometry measurements
            const float chiOdom() const {return _chi_odom;}
        
            //! chi square of the projection measurements
            const float chiProj() const {return _chi_proj;}

            void one_round();            

        protected:                    

            void errorAndJacobian_odom( int pose_i_idx, 
                                        int pose_j_idx, 
                                        Matrix6_3f& J_odom_pose_i,
                                        Matrix6_3f& J_odom_pose_j,
                                        Vector6f& odom_error);

            bool errorAndJacobian_proj( int pose_idx, 
                                        int landmark_idx,                                        
                                        Matrix2_3f& J_proj_landmark,
                                        Matrix2_3f& J_proj_pose,
                                        Eigen::Vector2f& proj_error);

            void boxplus();

            template <class Template_matrix>
            void fill_triplet_list(tripletList& triplets, Template_matrix& mat, int row, int column);

            float _proj_kernel_threshold;           //< threshold for projections kernel
            float _odom_kernel_threshold;           //< threshold for odometry kernel
            float _damping;                  //< damping, to slow the solution
            Camera _camera;                     //< this variable holds the camera parameters
            Vector3fVector _odometry_displacements; //< store all the measurements of the odometer as displacements between the positions 
            
            const MeasVector* _measurements;
            Vector3fVector* _odom_traj_estimate;
            Vector3fVector* _landmarks_estimate;

            Eigen::Isometry3f _camera_to_robot;
            
            Eigen::SparseMatrix<float> _H;
            
            Eigen::VectorXf _b;
            Eigen::VectorXf _delta_x;
            
            float _chi_odom;
            float _chi_proj;

            int _state_size;
            int _n_of_landmarks;
    };
}