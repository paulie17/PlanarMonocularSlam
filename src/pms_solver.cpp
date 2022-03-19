#include "pms_solver.h"
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
        _id_to_index_Map = 0;
    }

    void pms_solver::init(  Vector3dVector& odom_traj,
                            Vector3dVector& landmarks,
                            const MeasVector& measurements,
                            const std::map<int,int>& map ){         

        _state_size = odom_traj.size()*3 + map.size()*3;
        _damping=1;                        

        _odom_traj_estimate = &odom_traj;
        _landmarks_estimate = &landmarks;
        _measurements = &measurements;     
        _id_to_index_Map = &map;
        _n_of_landmarks = landmarks.size();   

        _H.resize(_state_size,_state_size);
        _b.resize(_state_size);  
        _delta_x.resize(_state_size);  

        for (int k = 1; k < NUM_MEASUREMENTS; k++) {
            // TODO: displ can be determined more efficiently with
            //       [R^T(theta1)(t1-t0); theta1-theta0].
            Eigen::Vector3d displ = t2v(v2t((*_measurements)[k-1].odom_pose).inverse() *
                                       (v2t((*_measurements)[k].odom_pose)));            
            _odometry_displacements.push_back(displ);           
        }    
    }

    bool pms_solver::errorAndJacobian_proj( int pose_idx, 
                                        int landmark_idx,                                        
                                        Matrix2_3d& J_proj_landmark,
                                        Matrix2_3d& J_proj_pose,
                                        Eigen::Vector2d& proj_error){
        bool is_inside;

        Matrix2_3d J_pi;

        Eigen::Vector3d landmark_estimate;
        Eigen::Vector3d odom_pose_estimate;
        Eigen::Vector3d p_hat;

        Eigen::Vector2d proj_prediction;
        Eigen::Vector2d proj_observation;                                                            

        Eigen::Isometry3d robot_to_world;

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
                                Matrix6_3d& J_odom_pose_i,
                                Matrix6_3d& J_odom_pose_j,
                                Vector6d& odom_error){

        Eigen::Vector3d pose_i;
        Eigen::Vector3d pose_j;

        Eigen::Matrix2d R_i;
        Eigen::Vector2d t_i;

        Eigen::Matrix2d R_j;
        Eigen::Vector2d t_j;

        Eigen::Isometry2d Z;

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
        Eigen::Vector3d pose_increment;

        for (int i = 0;i<_n_of_landmarks; i++){
                        
            if ( (*_id_to_index_Map).find(i) == ((*_id_to_index_Map).end()) ){
            continue;
            }

            landmark_idx = 3*NUM_MEASUREMENTS + (*_id_to_index_Map).at(i)*3; 
            (*_landmarks_estimate)[i] += _delta_x.segment<3>(landmark_idx);
        }

        for (int i = 0;i<NUM_MEASUREMENTS; i++){
            pose_idx = (i)*3;
            pose_increment = _delta_x.segment<3>(pose_idx);
            (*_odom_traj_estimate)[i] = t2v(v2t(pose_increment)*
                                            v2t((*_odom_traj_estimate)[i]));
        }
    }

    void pms_solver::fill_triplet_list(tripletList& triplets, const Eigen::MatrixXd& mat, int row, int column){
        int n_rows = mat.rows();
        int n_cols = mat.cols();

        //std::cout << mat << std::endl;
        for(int i = 0; i < n_rows; i++ ){
            for (int j = 0; j < n_cols; j++){
                //std::cout << "i: " << i+row << ", j:" << j+column << ", val: " << mat(i,j) << std::endl;
                triplets.push_back(Eigen::Triplet<double>(i+row,j+column,mat(i,j)));
            }
        }
    }

    void pms_solver::fill_sparse_values(const Eigen::MatrixXd& mat, int row, int column){
        int n_rows = mat.rows();
        int n_cols = mat.cols();

        //std::cout << mat << std::endl;
        for(int i = 0; i < n_rows; i++ ){
            for (int j = 0; j < n_cols; j++){
                //std::cout << "i: " << i+row << ", j:" << j+column << ", val: " << mat(i,j) << std::endl;
                _H.coeffRef(i+row,j+column) += mat(i,j);
            }
        }
    }


    void pms_solver::add_prior(tripletList& triplets){

        
        for(int j = 0; j < _H.cols(); j += 2){
            triplets.push_back(Eigen::Triplet<double>(0,j,1e12));
            triplets.push_back(Eigen::Triplet<double>(j,0,1e12));
        }

        for(int j = 1; j < _H.cols(); j += 2){
            triplets.push_back(Eigen::Triplet<double>(1,j,1e12));
            triplets.push_back(Eigen::Triplet<double>(j,1,1e12));
        }
        
        for(int j = 2; j < _H.cols(); j += 2){
            triplets.push_back(Eigen::Triplet<double>(2,j,1e12));
            triplets.push_back(Eigen::Triplet<double>(j,2,1e12));
        }

    }

    void pms_solver::one_round(){                
        
        _H.setZero();
        _b.setZero();  
        _delta_x.setZero(); 
        tripletList H_triplets;  
     
        // Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;    
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;

        int n_of_sensed_landmarks;
        int landmark_idx, pose_idx;                        

        int landmark_H_idx, pose_H_idx;

        int pose_i_idx, pose_j_idx;
        int pose_i_H_idx, pose_j_H_idx;

        bool is_valid;

        Matrix2_3d J_proj_landmark;
        Matrix2_3d J_proj_pose;
        Eigen::Vector2d proj_error;        

        Matrix6_3d J_odom_pose_i;
        Matrix6_3d J_odom_pose_j;
        Vector6d odom_error;                                     

        _chi_odom = 0.0;
        _chi_proj = 0.0;

        add_prior(H_triplets);
        
        // process the projection measurements first
        for(int i = 0; i<NUM_MEASUREMENTS; i++){
            n_of_sensed_landmarks = (*_measurements)[i].landmarks_img_pts.size();
            for (int j = 0; j < n_of_sensed_landmarks; j++){
                
                landmark_idx = (*_measurements)[i].detected_landmarks[j];
                // std::cout << landmark_idx << std::endl;
                if ( (*_id_to_index_Map).find(landmark_idx) == (*_id_to_index_Map).end() ){
                continue;
                }
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

                landmark_H_idx = 3*(NUM_MEASUREMENTS) + (*_id_to_index_Map).at(landmark_idx)*3;
                pose_H_idx = (pose_idx)*3;
                
                fill_triplet_list(H_triplets,J_proj_landmark.transpose()*J_proj_landmark,landmark_H_idx,landmark_H_idx);
                // fill_sparse_values(J_proj_landmark.transpose()*J_proj_landmark,landmark_H_idx,landmark_H_idx);
                //_H.block<3,3>(landmark_H_idx,landmark_H_idx) += J_proj_landmark.transpose()*J_proj_landmark;
                fill_triplet_list(H_triplets,J_proj_landmark.transpose()*J_proj_pose,landmark_H_idx,pose_H_idx);
                // fill_sparse_values(J_proj_landmark.transpose()*J_proj_pose,landmark_H_idx,pose_H_idx);
                //_H.block<3,3>(landmark_H_idx,pose_H_idx) += J_proj_landmark.transpose()*J_proj_pose;
                fill_triplet_list(H_triplets,J_proj_pose.transpose()*J_proj_pose,pose_H_idx,pose_H_idx);
                // fill_sparse_values(J_proj_pose.transpose()*J_proj_pose,pose_H_idx,pose_H_idx);
                //_H.block<3,3>(pose_H_idx,pose_H_idx) += J_proj_pose.transpose()*J_proj_pose;
                fill_triplet_list(H_triplets,J_proj_pose.transpose()*J_proj_landmark,pose_H_idx,landmark_H_idx);
                // fill_sparse_values(J_proj_pose.transpose()*J_proj_landmark,pose_H_idx,landmark_H_idx);
                //_H.block<3,3>(pose_H_idx,landmark_H_idx) += J_proj_pose.transpose()*J_proj_landmark;                       

                //fill_triplet_list(b_triplets,J_proj_pose.transpose() * proj_error,pose_H_idx,0);
                _b.segment<3>(pose_H_idx) += J_proj_pose.transpose() * proj_error;
                //fill_triplet_list(b_triplets,J_proj_landmark.transpose() * proj_error,landmark_H_idx,0);
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

            fill_triplet_list(H_triplets,J_odom_pose_i.transpose()*J_odom_pose_i,pose_i_H_idx,pose_i_H_idx);
            // fill_sparse_values(J_odom_pose_i.transpose()*J_odom_pose_i,pose_i_H_idx,pose_i_H_idx);

            //_H.block<3,3>(pose_i_H_idx,pose_i_H_idx) += J_odom_pose_i.transpose()*J_odom_pose_i;
            fill_triplet_list(H_triplets,J_odom_pose_i.transpose()*J_odom_pose_j,pose_i_H_idx,pose_j_H_idx);
            // fill_sparse_values(J_odom_pose_i.transpose()*J_odom_pose_j,pose_i_H_idx,pose_j_H_idx);

            //_H.block<3,3>(pose_i_H_idx,pose_j_H_idx) += J_odom_pose_i.transpose()*J_odom_pose_j;
            fill_triplet_list(H_triplets,J_odom_pose_j.transpose()*J_odom_pose_j,pose_j_H_idx,pose_j_H_idx);
            // fill_sparse_values(J_odom_pose_j.transpose()*J_odom_pose_j,pose_j_H_idx,pose_j_H_idx);

            //_H.block<3,3>(pose_j_H_idx,pose_j_H_idx) += J_odom_pose_j.transpose()*J_odom_pose_j;
            fill_triplet_list(H_triplets,J_odom_pose_j.transpose()*J_odom_pose_i,pose_j_H_idx,pose_i_H_idx);
            // fill_sparse_values(J_odom_pose_j.transpose()*J_odom_pose_i,pose_j_H_idx,pose_i_H_idx);

            //_H.block<3,3>(pose_j_H_idx,pose_i_H_idx) += J_odom_pose_j.transpose()*J_odom_pose_i;                  
            
            //fill_triplet_list(b_triplets,J_odom_pose_i.transpose() * odom_error,pose_i_H_idx,0);
            _b.segment<3>(pose_i_H_idx) += J_odom_pose_i.transpose() * odom_error;
            //fill_triplet_list(b_triplets,J_odom_pose_j.transpose() * odom_error,pose_j_H_idx,0);
            _b.segment<3>(pose_j_H_idx) += J_odom_pose_j.transpose() * odom_error;            
        }
        
        _H.setFromTriplets(H_triplets.begin(),H_triplets.end());
        _H.makeCompressed();
        //_b.setFromTriplets(b_triplets.begin(),b_triplets.end());
        
        solver.compute(_H);
        // solver.analyzePattern(_H);   // for this step the numerical values of A are not used
        // solver.factorize(_H);
        if(solver.info()!=Eigen::Success) {
        // decomposition failed
        std::cout << "Decomposition failed! \n";
        }
        _delta_x = solver.solve(-_b);       
        if(solver.info()!=Eigen::Success) {
        std::cout << "Solving failed! \n";        
        }
        // std::cout << _b << std::endl;
        // std::cin.get();
        //_H +=Eigen::MatrixXd::Identity(_state_size,_state_size)*_damping;
        //_delta_x.tail += _H.ldlt().solve(-_b);
        //if (lock_poses){
        //    const int subsys_size = _state_size - 3*NUM_MEASUREMENTS;
        //    _delta_x.tail(subsys_size) = _H.bottomRightCorner(subsys_size, subsys_size).ldlt().solve(-_b.tail(subsys_size));
        //}
        //else
        //{
        //const int subsys_size = _state_size - 3;
        //_delta_x.tail(subsys_size) = _H.bottomRightCorner(subsys_size, subsys_size).ldlt().solve(-_b.tail(subsys_size));       
        //}
        
        boxplus();

    }
}