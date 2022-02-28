#include "dataset_utils.h"

namespace pms{

    void load_trajectory(Vector3fVector& odom_trajectory, Vector3fVector& gt_trajectory){
        
        std::istringstream ss;
        std::string dummy_string;

        int current_pose;
        //float dummy_var;
          
        Eigen::Vector3f odom_pose;
        Eigen::Vector3f gt_pose;

        std::ifstream trajectory_file("../runba_sim/trajectory.dat");    

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

    void load_camera_data(  int& z_near,int& z_far, 
                            int& width, int& height,
                            Eigen::Matrix3f& camera_matrix, Eigen::Isometry3f& camera_to_robot){
        
        std::string dummy_string;

        Eigen::Matrix4f camera_Transform;
        Eigen::Vector3f t;
        Eigen::Matrix3f R;

        std::ifstream camera_file("../runba_sim/camera.dat");
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

    void load_measurements(MeasVector& measurements){        
    
        Measurement meas;
        std::string dummy_string,point_string;

        Eigen::Vector2f img_pt;  
        Vector10f appearance;
        int pt_idx,landmark_id_gt;  

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
        
        
        for (int k = 0; k < NUM_MEASUREMENTS; k++){
            
            ss << std::setw(5) << std::setfill('0') << k;
            measurement_filename = "../runba_sim/meas-" + ss.str() + ".dat";
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
                    point_stream >> dummy_string >> pt_idx >> landmark_id_gt >> img_pt(0) >> img_pt(1);
                    for (int i = 0;i < 10; i++){
                        point_stream >> appearance(i);
                    }                    
                    meas.detected_landmarks.push_back(-1);
                    meas.appearances.push_back(appearance);
                    meas.landmarks_img_pts.push_back(img_pt);
                    point_stream.clear();
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
    }

    Vector3fVector load_landmarks_gt(){
        std::string dummy_string;
        int current_idx;

        Vector3fVector landmarks_gt;  
        Eigen::Vector3f current_landmark;

        std::ifstream world_file("../runba_sim/world.dat");    

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

    Vector3fVector initial_guess(MeasVector& measurements,int NUM_LANDMARKS, IntVector& discarded) {
        
        // algorithm from
        // Slabaugh, Greg, Ron Schafer, and Mark Livingston. "Optimal ray intersection for computing 3d points from n-view correspondences." 
        // Deliverable Report (2001): 1-11.

        Eigen::Vector3f current_bearing;
        Eigen::Vector3f current_camera;

        std::vector<int>::iterator it;
        int k;

        int n_of_corrispondences;

        Eigen::Matrix3f A;
        Eigen::Vector3f b;
        Eigen::Vector3f solution;

        Vector3fVector landmarks_initial_guess;

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

    void match_features(const Vector10fVector& desc1,
                    const Vector10fVector& desc2,
                    const double ratio,
                    IntPairVector& matches) {
        // Find all desc1 -> desc2 matches.
        typedef std::pair<float, int> MatchDistance;
        IntPairVector matches_;
        std::vector<std::vector<MatchDistance> > match_distances(desc1.size());
        
        for (int i = 0; i < desc1.size(); i++) {
            match_distances[i].resize(desc2.size());
            for (int j = 0; j < desc2.size(); j++) {
            const float distance = (desc1[i] - desc2[j]).squaredNorm();
            match_distances[i][j] = std::make_pair(distance, j);
            }
        }
        // Only save the matches that pass the lowest ratio test.
        matches_.reserve(desc1.size());
        for (int i = 0; i < match_distances.size(); i++) {
            // Get the top 2 matches.
            std::partial_sort(match_distances[i].begin(),
                            match_distances[i].begin() + 2,
                            match_distances[i].end());
            if (match_distances[i][0].first / match_distances[i][1].first < ratio) {
            matches_.push_back(std::make_pair(i, match_distances[i][0].second));
            }
        }
        matches.swap(matches_);
    }

    int data_association(MeasVector& measurements){
	
        int landmark_count = 0;
        //Vector10fVector  desc1 = measurements[0].appearances;
        //Vector10fVector  desc2 = measurements[1].appearances;

        IntPairVector matches;
        IntPairVector unassigned_points;
        Vector10fVector unassigned_appearances;
        float dratio = 0.8f;
        
        match_features(measurements[1].appearances, measurements[0].appearances,dratio,matches);	
        //now in matches we have a vector of pairs,
        //each pair being a correspondence of an appearance in desc1
        //with an appearance in desc2. I should use these pairs and
        //record the result in the measurements' structs.

        for (int i = 0; i < matches.size(); i++){
            measurements[1].detected_landmarks[matches[i].first] = landmark_count;
            measurements[0].detected_landmarks[matches[i].second] = landmark_count;
            landmark_count ++;
        }	

        std::vector<int>::iterator iter = measurements[1].detected_landmarks.begin();
        int index;

        while ((iter = std::find(iter,measurements[1].detected_landmarks.end(),-1)) != measurements[1].detected_landmarks.end()){
            index = distance(measurements[1].detected_landmarks.begin(),iter);
            unassigned_points.push_back(std::make_pair(1,index));
            unassigned_appearances.push_back(measurements[1].appearances[index]);
            iter++;
        }	

        iter = measurements[0].detected_landmarks.begin();
        
        while ((iter = std::find(iter,measurements[0].detected_landmarks.end(),-1)) != measurements[0].detected_landmarks.end()){
            index = distance(measurements[0].detected_landmarks.begin(),iter);
            unassigned_points.push_back(std::make_pair(0,index));
            unassigned_appearances.push_back(measurements[0].appearances[index]);
            iter++;
        }	
        
        if (measurements.size() > 2) {

            for (int i = 2; i < measurements.size(); i ++){
                std::cout << i << "\n";
                // first check matchings with previous measurement
                match_features(measurements[i].appearances,measurements[i-1].appearances,dratio,matches);
                for (int j=0;j<matches.size();j++){
                    
                    if(measurements[i-1].detected_landmarks[matches[j].second] != -1){
                        measurements[i].detected_landmarks[matches[j].first] = measurements[i-1].detected_landmarks[matches[j].second];}
                    else{
                        measurements[i].detected_landmarks[matches[j].first] = landmark_count;
                        measurements[i-1].detected_landmarks[matches[j].second] = landmark_count;
                        landmark_count ++;}
                }	
                // if there are points that haven't been matched yet then one should look at the other meaasurements (from i-2 up to 0)
                if (unassigned_appearances.size() > 0){
                    match_features(measurements[i].appearances,unassigned_appearances,dratio,matches);
                    for (int j = 0; j < matches.size(); j++){
                        measurements[i].detected_landmarks[matches[j].first] = landmark_count;
                        measurements[unassigned_points[matches[j].second].first].detected_landmarks[unassigned_points[matches[j].second].second] = landmark_count;
                        unassigned_points.erase(unassigned_points.begin()+matches[j].second);
                        unassigned_appearances.erase(unassigned_appearances.begin()+matches[j].second);
                        landmark_count ++;
                    }
                }
                
                iter = measurements[i].detected_landmarks.begin();
        
                while ((iter = std::find(iter,measurements[i].detected_landmarks.end(),-1)) != measurements[i].detected_landmarks.end()){
                    index = distance(measurements[i].detected_landmarks.begin(),iter);
                    unassigned_points.push_back(std::make_pair(i,index));
                    unassigned_appearances.push_back(measurements[i].appearances[index]);
                    iter++;
                }

            }
        }
        
        for (int i = 0; i < unassigned_points.size(); i++){
            measurements[unassigned_points[i].first].appearances.erase(measurements[unassigned_points[i].first].appearances.begin() + 
                                                                                unassigned_points[i].second);
            measurements[unassigned_points[i].first].detected_landmarks.erase(measurements[unassigned_points[i].first].detected_landmarks.begin() + 
                                                                                unassigned_points[i].second);
        }
    return landmark_count+1;
    }
    
}