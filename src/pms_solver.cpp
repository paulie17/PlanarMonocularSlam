#include "pms_solver.h"

#include <Eigen/Cholesky>
#include <iostream>
#include <cmath>

namespace pms {

    pms_solver::pms_solver(){
        //_kernel_thereshold = 0.1;           
        _camera = load_camera_data();
        _camera_to_robot = _camera.cameraToWorldPose(); 
        _measurements = 0;
        _odom_traj_estimate = 0;
        _landmarks_estimate = 0;        
    }

    void pms_solver::init(  Vector3fVector& odom_traj,
                            Vector3fVector& landmarks,
                            const MeasVector& measurements){         

        _state_size = odom_traj.size()*3 + landmarks.size()*3;
        _damping=1;
        _n_of_landmarks = landmarks.size();                   

        _odom_traj_estimate = &odom_traj;
        _landmarks_estimate = &landmarks;
        _measurements = &measurements;     

        _H.resize(_state_size,_state_size);
        _b.resize(_state_size);  
        _delta_x.resize(_state_size);  

        for (int k = 1; k < NUM_MEASUREMENTS; k++) {
            // TODO: displ can be determined more efficiently with
            //       [R^T(theta1)(t1-t0); theta1-theta0].
            Eigen::Vector3f displ = t2v(v2t((*_measurements)[k-1].odom_pose).inverse() *
                                       (v2t((*_measurements)[k].odom_pose)));            
            _odometry_displacements.push_back(displ);           
        }    
    }

    bool pms_solver::errorAndJacobian_proj( int pose_idx, 
                                        int landmark_idx,                                        
                                        Matrix2_3f& J_proj_landmark,
                                        Matrix2_3f& J_proj_pose,
                                        Eigen::Vector2f& proj_error){
        bool is_inside;

        Matrix2_3f J_pi;

        Eigen::Vector3f landmark_estimate;
        Eigen::Vector3f odom_pose_estimate;
        Eigen::Vector3f p_hat;

        Eigen::Vector2f proj_prediction;
        Eigen::Vector2f proj_observation;                                                            

        Eigen::Isometry3f robot_to_world;

        std::vector<int>::const_iterator j;       
        int k;                 
        
        J_proj_landmark.setZero();
        J_proj_pose.setZero();

        odom_pose_estimate = (*_odom_traj_estimate)[pose_idx];  
        landmark_estimate = (*_landmarks_estimate)[landmark_idx];   

        if (isNan(landmark_estimate)){
            return false;
        }

        robot_to_world = t2t3d(v2t(odom_pose_estimate));
        _camera.setCameraToWorldPose(robot_to_world*_camera_to_robot);     

        p_hat = _camera.cameraMatrix()*_camera.cameraToWorldPose().inverse()*landmark_estimate;

        is_inside = _camera.projectPoint(proj_prediction,landmark_estimate);
        
        if (!is_inside){
            return is_inside;
        }

        j = std::find ( (*_measurements)[pose_idx].detected_landmarks.begin(), 
                        (*_measurements)[pose_idx].detected_landmarks.end(), 
                        landmark_idx);
        k = distance((*_measurements)[pose_idx].detected_landmarks.begin(),j); 

        proj_observation = (*_measurements)[pose_idx].landmarks_img_pts[k]; 

        proj_error = proj_prediction - proj_observation;  

        /*std::cout << "This is the error of the projection of landmark " << landmark_idx <<" from pose " << pose_idx << "\n";
        std::cout << proj_error << "\n";
        std::cin.get();     */                                    
        
        J_pi << 1.0f/p_hat.z(), 0.0f, -p_hat.x()/std::pow(p_hat.z(), 2.0f),
                0.0f, 1.0f/p_hat.z(), -p_hat.y()/std::pow(p_hat.z(), 2.0f);
        J_proj_pose.block<2,2>(0,0) = -J_pi*_camera.cameraMatrix()*_camera.cameraToWorldPose().inverse().linear().block<3,2>(0,0);
        J_proj_pose.block<2,1>(0,2) = J_pi*_camera.cameraMatrix()*_camera.cameraToWorldPose().inverse().linear()*dRz(0).inverse()*(landmark_estimate);          
        J_proj_landmark = J_pi*_camera.cameraMatrix()*_camera.cameraToWorldPose().inverse().linear();

        return is_inside;
    }

    void pms_solver::errorAndJacobian_odom( int pose_i_idx, 
                                int pose_j_idx, 
                                Matrix6_3f& J_odom_pose_i,
                                Matrix6_3f& J_odom_pose_j,
                                Vector6f& odom_error){

        Eigen::Vector3f pose_i;
        Eigen::Vector3f pose_j;

        Eigen::Matrix2f R_i;
        Eigen::Vector2f t_i;

        Eigen::Matrix2f R_j;
        Eigen::Vector2f t_j;

        Eigen::Isometry2f Z;

        J_odom_pose_j.setZero();

        pose_i = (*_odom_traj_estimate)[pose_i_idx]; 
        pose_j = (*_odom_traj_estimate)[pose_j_idx];

        R_i = v2t(pose_i).linear();        
        R_j = v2t(pose_j).linear();
        t_j = v2t(pose_j).translation();

        Z = v2t(_odometry_displacements[pose_i_idx]);     

        J_odom_pose_j.block<2,2>(4,0) += R_i.transpose();
        J_odom_pose_j.block<4,1>(0,2) += flatten(R_i.transpose()*dR_2d(0)*R_j);
        J_odom_pose_j.block<2,1>(4,2) += -R_i.transpose()*dR_2d(0)*t_j;
        J_odom_pose_i = -J_odom_pose_j;

        odom_error = flatten((v2t(pose_i).inverse()*v2t(pose_j)).matrix().block<2,3>(0,0))
                        -flatten(Z.matrix().block<2,3>(0,0));    
        //std::cout << odom_error << "\n";
        //std::cin.get();
    }    

    void pms_solver::boxplus(){

        int landmark_idx, pose_idx;
        Eigen::Vector3f pose_increment;

        for (int i = 0;i<_n_of_landmarks; i++){
            landmark_idx = 3*NUM_MEASUREMENTS + (i)*3; 
            
            if (isNan((*_landmarks_estimate)[i])){
            continue;
            }

            (*_landmarks_estimate)[i] += _delta_x.segment<3>(landmark_idx);
        }

        for (int i = 0;i<NUM_MEASUREMENTS; i++){
            pose_idx = (i)*3;
            pose_increment = _delta_x.segment<3>(pose_idx);
            (*_odom_traj_estimate)[i] = t2v(v2t(pose_increment)*
                                            v2t((*_odom_traj_estimate)[i]));
        }
    }

    void pms_solver::one_round(){                
        
        _H.setZero();
        _b.setZero();  
        _delta_x.setZero(); 

        int n_of_sensed_landmarks;
        int landmark_idx, pose_idx;                        

        int landmark_H_idx, pose_H_idx;

        int pose_i_idx, pose_j_idx;
        int pose_i_H_idx, pose_j_H_idx;

        bool is_valid;

        Matrix2_3f J_proj_landmark;
        Matrix2_3f J_proj_pose;
        Eigen::Vector2f proj_error;        

        Matrix6_3f J_odom_pose_i;
        Matrix6_3f J_odom_pose_j;
        Vector6f odom_error;                                     

        _chi_odom = 0.0;
        _chi_proj = 0.0;

        // process the projection measurements first
        for(int i = 0; i<NUM_MEASUREMENTS; i++){
            n_of_sensed_landmarks = (*_measurements)[i].landmarks_img_pts.size();
            for (int j = 0; j < n_of_sensed_landmarks; j++){

                landmark_idx = (*_measurements)[i].detected_landmarks[j];
                pose_idx = (*_measurements)[i].seq;      

                is_valid = errorAndJacobian_proj( pose_idx, landmark_idx,
                                        J_proj_landmark,J_proj_pose,proj_error);
                if (!is_valid){
                    continue;
                }
                if (proj_error.squaredNorm() > _proj_kernel_threshold) {
                       proj_error *= std::sqrt(_proj_kernel_threshold / proj_error.squaredNorm());
                       _chi_proj += _proj_kernel_threshold;
                    }
                else
                {_chi_proj += proj_error.transpose()*proj_error;} 

                landmark_H_idx = 3*(NUM_MEASUREMENTS) + (landmark_idx)*3;
                pose_H_idx = (pose_idx)*3;

                _H.block<3,3>(landmark_H_idx,landmark_H_idx) += J_proj_landmark.transpose()*J_proj_landmark;
                _H.block<3,3>(landmark_H_idx,pose_H_idx) += J_proj_landmark.transpose()*J_proj_pose;
                _H.block<3,3>(pose_H_idx,pose_H_idx) += J_proj_pose.transpose()*J_proj_pose;
                _H.block<3,3>(pose_H_idx,landmark_H_idx) += J_proj_pose.transpose()*J_proj_landmark;                       

                _b.segment<3>(pose_H_idx) += J_proj_pose.transpose() * proj_error;
                _b.segment<3>(landmark_H_idx) += J_proj_landmark.transpose() * proj_error;

            }
        }        
                        
        for (int i = 0; i < (NUM_MEASUREMENTS-1); i++){
            
            pose_i_idx = i;
            pose_j_idx = i + 1;

            errorAndJacobian_odom( pose_i_idx, pose_j_idx, J_odom_pose_i, J_odom_pose_j, odom_error);
            
            if (odom_error.squaredNorm() > _odom_kernel_threshold) {
                    odom_error *= std::sqrt(_odom_kernel_threshold / odom_error.squaredNorm());
                    _chi_odom += _odom_kernel_threshold;
                }
            else
            {_chi_odom += odom_error.transpose()*odom_error;} 

            pose_i_H_idx =  (pose_i_idx)*3;
            pose_j_H_idx =  (pose_j_idx)*3;

            _H.block<3,3>(pose_i_H_idx,pose_i_H_idx) += J_odom_pose_i.transpose()*J_odom_pose_i;
            _H.block<3,3>(pose_i_H_idx,pose_j_H_idx) += J_odom_pose_i.transpose()*J_odom_pose_j;
            _H.block<3,3>(pose_j_H_idx,pose_j_H_idx) += J_odom_pose_j.transpose()*J_odom_pose_j;
            _H.block<3,3>(pose_j_H_idx,pose_i_H_idx) += J_odom_pose_j.transpose()*J_odom_pose_i;                  
            
            _b.segment<3>(pose_i_H_idx) += J_odom_pose_i.transpose() * odom_error;
            _b.segment<3>(pose_j_H_idx) += J_odom_pose_j.transpose() * odom_error;
            
        }
        
        
        //_H +=Eigen::MatrixXf::Identity(_state_size,_state_size)*_damping;
        //_delta_x.tail += _H.ldlt().solve(-_b);
        //if (lock_poses){
        //    const int subsys_size = _state_size - 3*NUM_MEASUREMENTS;
        //    _delta_x.tail(subsys_size) = _H.bottomRightCorner(subsys_size, subsys_size).ldlt().solve(-_b.tail(subsys_size));
        //}
        //else
        //{
        const int subsys_size = _state_size - 3;
        _delta_x.tail(subsys_size) = _H.bottomRightCorner(subsys_size, subsys_size).ldlt().solve(-_b.tail(subsys_size));       
        //}
        
        boxplus();

    }
}