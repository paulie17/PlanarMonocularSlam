#include "defs.h"
#include "camera.h"
#include "dataset_utils.h"
#include <cmath>

namespace pr {
    inline Vector3fVector initial_guess(MeasVector measurements,int NUM_LANDMARKS) {
        
        Eigen::Vector3f current_bearing;
        Eigen::Vector3f current_camera;

        std::vector<int>::iterator it;

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
                    //std::cout << "Landmark " << i << " observed in measurement " << j << " with index " << k << std::endl;
                                   
                    current_bearing = measurements[j].bearings[*it];
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