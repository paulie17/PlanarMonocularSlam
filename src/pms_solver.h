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

            void init(  Vector3dVector& odom_traj,
                        Vector3dVector& landmarks,
                        const MeasVector& measurements,
                        const std::map<int,int>& map);
    
            inline double ProjkernelThreshold() const {return _proj_kernel_threshold;}

            inline void setProjKernelThreshold(double kernel_threshold) 
            {_proj_kernel_threshold=kernel_threshold;}

            inline double OdomkernelThreshold() const {return _odom_kernel_threshold;}

            inline void setOdomKernelThreshold(double kernel_threshold) 
            {_odom_kernel_threshold=kernel_threshold;}

            //! chi square of the odometry measurements
            const double chiOdom() const {return _chi_odom;}
        
            //! chi square of the projection measurements
            const double chiProj() const {return _chi_proj;}

            void one_round();            

        protected:                    

            void errorAndJacobian_odom( int pose_i_idx, 
                                        int pose_j_idx, 
                                        Matrix6_3d& J_odom_pose_i,
                                        Matrix6_3d& J_odom_pose_j,
                                        Vector6d& odom_error);

            bool errorAndJacobian_proj( int pose_idx, 
                                        int landmark_idx,                                        
                                        Matrix2_3d& J_proj_landmark,
                                        Matrix2_3d& J_proj_pose,
                                        Eigen::Vector2d& proj_error);

            void boxplus();
            void add_prior(tripletList& triplets);
            void fill_triplet_list(tripletList& triplets, const Eigen::MatrixXd& mat, int row, int column);
            void fill_sparse_values(const Eigen::MatrixXd& mat, int row, int column);

            double _proj_kernel_threshold;           //< threshold for projections kernel
            double _odom_kernel_threshold;           //< threshold for odometry kernel
            double _damping;                  //< damping, to slow the solution
            Camera _camera;                     //< this variable holds the camera parameters
            Vector3dVector _odometry_displacements; //< store all the measurements of the odometer as displacements between the positions 
            
            const MeasVector* _measurements;
            Vector3dVector* _odom_traj_estimate;
            Vector3dVector* _landmarks_estimate;
            const std::map<int,int>* _id_to_index_Map;

            Eigen::Isometry3d _camera_to_robot;
            
            Eigen::SparseMatrix<double> _H;
            
            Eigen::VectorXd _b;
            Eigen::VectorXd _delta_x;
            
            double _chi_odom;
            double _chi_proj;

            int _state_size;
            int _n_of_landmarks;
    };
}