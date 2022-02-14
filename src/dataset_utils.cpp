#include "dataset_utils.h"

namespace pr{

    Vector3fVector load_trajectory(){
        
        std::istringstream ss;
        std::string dummy_string;

        int current_pose;
        //float dummy_var;

        Vector3fVector odom_trajectory;  
        Eigen::Vector3f odom_pose;

        std::ifstream trajectory_file("../dataset/trajectory.dat");    

        while (std::getline(trajectory_file,dummy_string)){
            ss.str(dummy_string);
            ss >> dummy_string >> current_pose;
            
            for (int i = 0; i<3; i++){
                ss >> odom_pose(i);                
            }                    
            odom_trajectory.push_back(odom_pose);

            //ss >> dummy_var >> dummy_var >> dummy_var;
            ss.clear();
        }
        return odom_trajectory;
    }

    void load_camera_data(  int& z_near,int& z_far, 
                            int& width, int& height,
                            Eigen::Matrix3f& camera_matrix, Eigen::Isometry3f& camera_to_robot){
        
        std::string dummy_string;

        Eigen::Matrix4f camera_Transform;
        Eigen::Vector3f t;
        Eigen::Matrix3f R;

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
        int z_near, z_far, width, height;
        Eigen::Matrix3f camera_matrix;
        Eigen::Isometry3f camera_to_robot;

        load_camera_data(z_near,z_far,width,height,camera_matrix,camera_to_robot);

        cam.setCameraMatrix(camera_matrix);
        cam.setImageSize(height,width);

        // set camera_to_robot as the pose matrix for the moment;
        // later in the solver you can rotate/translate using the robot position.
        cam.setCameraToWorldPose(camera_to_robot);

        return cam;
    }

    int load_measurements(MeasVector& measurements){        
    
        Measurement meas;
        std::string dummy_string,point_string;

        Eigen::Vector2f img_pt;  
        int pt_idx,landmark_id;  

        std::ifstream measurement_file;
        
        std::stringstream ss;
        std::istringstream point_stream;
        std::string measurement_filename;

        //Declare variables to import camera data
        Eigen::Matrix3f cameraMatrix;
        Eigen::Matrix4f camera_Transform;
        Eigen::Isometry3f camera_to_robot;

        Eigen::Isometry2f robot_to_world_2d;
        Eigen::Isometry3f robot_to_world_3d;
        
        Eigen::Isometry3f camera_to_world;


        int z_near,z_far;
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
            measurements[i].bearings = measurements[i].current_camera_position.bearings_from_img_points(measurements[i].landmarks_img_pts);
        }
        
        return n_of_landmarks;
    }

    Vector3fVector load_landmarks_gt(){
        std::string dummy_string;
        int current_idx;

        Vector3fVector landmarks_gt;  
        Eigen::Vector3f current_landmark;

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
}