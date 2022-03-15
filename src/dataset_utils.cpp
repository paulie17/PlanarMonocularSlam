#include "dataset_utils.h"

namespace pms{

    void load_trajectory(Vector3dVector& odom_trajectory, Vector3dVector& gt_trajectory){
        
        std::istringstream ss;
        std::string dummy_string;

        int current_pose;
        //float dummy_var;
          
        Eigen::Vector3d odom_pose;
        Eigen::Vector3d gt_pose;

        std::ifstream trajectory_file("../dataset/trajectory.dat");    

        while (std::getline(trajectory_file,dummy_string)){
            ss.str(dummy_string);
            ss >> dummy_string >> current_pose;
            
            for (int i = 0; i<3; i++){
                ss >> odom_pose(i);                
            }       
            for (int i = 0; i<3; i++){
                ss >> gt_pose(i);                
            }                
            odom_trajectory.push_back(odom_pose);
            gt_trajectory.push_back(gt_pose);

            //ss >> dummy_var >> dummy_var >> dummy_var;
            ss.clear();
        }
    }

    void load_camera_data(  double& z_near,double& z_far, 
                            int& width, int& height,
                            Eigen::Matrix3d& camera_matrix, Eigen::Isometry3d& camera_to_robot){
        
        std::string dummy_string;

        Eigen::Matrix4d camera_Transform;
        Eigen::Vector3d t;
        Eigen::Matrix3d R;

        std::ifstream camera_file("../dataset/camera.dat");
        std::getline(camera_file, dummy_string);
        
        for (int i = 0; i < 3; i ++) {
            for (int j = 0; j<3; j++){
                camera_file >> camera_matrix(i,j);
            }    
        }

        camera_file >> dummy_string;
        for (int i = 0; i < 4; i ++) {
            for (int j = 0; j<4; j++){
                camera_file >> camera_Transform(i,j);
            }    
        }
              
        camera_file >> dummy_string >> z_near;
        camera_file >> dummy_string >> z_far;
        camera_file >> dummy_string >> width;
        camera_file >> dummy_string >> height;
        
        t = camera_Transform.block(0,3,3,1);
        R = camera_Transform.block(0,0,3,3);

        camera_to_robot.setIdentity();        
        camera_to_robot.prerotate(R); 
        camera_to_robot.translation() = t;               
    }

    Camera load_camera_data(){
        Camera cam;
        double z_near, z_far;
        int width, height;
        Eigen::Matrix3d camera_matrix;
        Eigen::Isometry3d camera_to_robot;

        load_camera_data(z_near,z_far,width,height,camera_matrix,camera_to_robot);

        cam.setCameraMatrix(camera_matrix);
        cam.setImageSize(height,width);
        cam.setDistanceLimits(z_near,z_far);
        // set camera_to_robot as the pose matrix for the moment;
        // later in the solver you can rotate/translate using the robot position.
        cam.setCameraToWorldPose(camera_to_robot);

        return cam;
    }

    int load_measurements(MeasVector& measurements){        
    
        Measurement meas;
        std::string dummy_string,point_string;

        Eigen::Vector2d img_pt;  
        int pt_idx,landmark_id;  

        std::ifstream measurement_file;
        
        std::stringstream ss;
        std::istringstream point_stream;
        std::string measurement_filename;

        //Declare variables to import camera data
        Eigen::Matrix3d cameraMatrix;
        Eigen::Matrix4d camera_Transform;
        Eigen::Isometry3d camera_to_robot;

        Eigen::Isometry2d robot_to_world_2d;
        Eigen::Isometry3d robot_to_world_3d;
        
        Eigen::Isometry3d camera_to_world;


        double z_near,z_far;
        int width, height;
        int n_of_landmarks = 0;
        
        
        for (int k = 0; k < NUM_MEASUREMENTS; k++){
            
            ss << std::setw(5) << std::setfill('0') << k;
            measurement_filename = "../dataset/meas-" + ss.str() + ".dat";
            //cout << measurement_filename << endl;
            measurement_file.open(measurement_filename);
            measurement_file >> dummy_string >> meas.seq;
            measurement_file >> dummy_string;
            for (int i = 0; i<3; i++){
                measurement_file >> meas.gt_pose(i);
            } 
            measurement_file >> dummy_string;
            for (int i = 0; i<3; i++){
                measurement_file >> meas.odom_pose(i);
            } 

            meas.detected_landmarks.clear();
            meas.landmarks_img_pts.clear();

            while(std::getline(measurement_file,point_string)){
                if (point_string.find("point") == 0){
                    point_stream.str(point_string);
                    point_stream >> dummy_string >> pt_idx >> landmark_id >> img_pt(0) >> img_pt(1);
                    meas.detected_landmarks.push_back(landmark_id);
                    meas.landmarks_img_pts.push_back(img_pt);
                    point_stream.clear();
                    if(landmark_id>n_of_landmarks){
                        n_of_landmarks = landmark_id;
                    }
                }
            }

            measurements.push_back(meas);
            measurement_file.close();
            ss.str("");
        }
                
        load_camera_data(z_near,z_far,width,height,cameraMatrix,camera_to_robot);     

        for (int i = 0; i < NUM_MEASUREMENTS; i++){
            robot_to_world_2d = v2t(measurements[i].odom_pose);
            robot_to_world_3d = t2t3d(robot_to_world_2d);            
            camera_to_world = robot_to_world_3d * camera_to_robot;
            measurements[i].current_camera_position.setCameraToWorldPose(camera_to_world);
            measurements[i].current_camera_position.setImageSize(height,width);
            measurements[i].current_camera_position.setCameraMatrix(cameraMatrix);
            measurements[i].current_camera_position.setDistanceLimits(z_near,z_far);
            measurements[i].bearings = measurements[i].current_camera_position.bearings_from_img_points(measurements[i].landmarks_img_pts);
        }
        
        return n_of_landmarks+1;
    }

    Vector3dVector load_landmarks_gt(){
        std::string dummy_string;
        int current_idx;

        Vector3dVector landmarks_gt;  
        Eigen::Vector3d current_landmark;

        std::ifstream world_file("../dataset/world.dat");    

        std::istringstream ss;

        while (std::getline(world_file,dummy_string)){
            ss.str(dummy_string);
            ss >> current_idx;
            
            for (int i = 0; i<3; i++){
                ss >> current_landmark(i);
            }                    
            landmarks_gt.push_back(current_landmark);
            ss.clear();
        }
        return landmarks_gt;
    }

    Vector3dVector initial_guess(MeasVector& measurements,int NUM_LANDMARKS, IntVector& discarded) {
        
        // algorithm from
        // Slabaugh, Greg, Ron Schafer, and Mark Livingston. "Optimal ray intersection for computing 3d points from n-view correspondences." 
        // Deliverable Report (2001): 1-11.

        Eigen::Vector3d current_bearing;
        Eigen::Vector3d current_camera;

        std::vector<int>::iterator it;
        int k;

        int n_of_corrispondences;

        Eigen::Matrix3d A;
        Eigen::Vector3d b;
        Eigen::Vector3d solution;

        Vector3dVector landmarks_initial_guess;

        A.setZero();
        b.setZero();

        for(int i = 0; i < NUM_LANDMARKS; i ++){
            //for each landmark i
            n_of_corrispondences = 0;

            for (int j = 0; j < NUM_MEASUREMENTS; j++){
                //check every measurement j
                it = std::find(measurements[j].detected_landmarks.begin(),
                                measurements[j].detected_landmarks.end(),i);  

                if (it != measurements[j].detected_landmarks.end()){
                    n_of_corrispondences ++;    
                    k = distance(measurements[j].detected_landmarks.begin(),it);                
                    //std::cout << "Landmark " << i << " observed in measurement " << j << " with index " << k << std::endl;
                                   
                    current_bearing = measurements[j].bearings[k];
                    current_camera = measurements[j].current_camera_position.cameraToWorldPose().translation();                                        
            
                    A(0,0) += 1-current_bearing(0)*current_bearing(0);
                    A(0,1) -= current_bearing(0)*current_bearing(1);
                    A(0,2) -= current_bearing(0)*current_bearing(2);
                    A(1,1) += 1-current_bearing(1)*current_bearing(1);
                    A(1,2) -= current_bearing(1)*current_bearing(2);
                    A(2,2) += 1-current_bearing(2)*current_bearing(2);
                    A(1,0) = A(0,1);
                    A(2,0) = A(0,2);
                    A(2,1) = A(1,2);                    

                    b(0) += (1- current_bearing(0)*current_bearing(0))*current_camera(0) 
                            - current_bearing(0)*current_bearing(1)*current_camera(1)
                            - current_bearing(0)*current_bearing(2)*current_camera(2);
                    b(1) += - current_bearing(0)*current_bearing(1)*current_camera(0)   
                            + (1-current_bearing(1)*current_bearing(1))*current_camera(1)
                            - current_bearing(1)*current_bearing(2)*current_camera(2);
                    b(2) += - current_bearing(0)*current_bearing(2)*current_camera(0)
                            - current_bearing(1)*current_bearing(2)*current_camera(1)
                            + (1-current_bearing(2)*current_bearing(2))*current_camera(2);
                    //std::cout << b << std::endl;
                    //std::cin.get(); 
                }                
            }

            if(n_of_corrispondences<2){
                //std::cout << "Landmark: "<< i << " has to be discarded, since there are "<< n_of_corrispondences << " corrispondences. \n";
                discarded.push_back(i);
                solution << NAN,NAN,NAN;
            }
            else{
            solution = A.colPivHouseholderQr().solve(b);                 
            }
            landmarks_initial_guess.push_back(solution);

            A.setZero();
            b.setZero();
        }
        return landmarks_initial_guess;
    }
}