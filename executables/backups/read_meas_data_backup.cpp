#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include "defs.h"
#include "meas.h"

#include "gnuplot-iostream.h"

using namespace std;
using namespace Eigen;
using namespace pr;

const int NUM_MEASUREMENTS = 200;

int main(){

    vector<Measurement> measurements;
    
    Measurement meas;
    string dummy_string,point_string;

    Vector2f img_pt;  
    int pt_idx,landmark_id;  

    ifstream measurement_file;
    
    stringstream ss;
    istringstream point_stream;
    string measurement_filename;

    for (int k = 0; k < NUM_MEASUREMENTS; k++){
        
        ss << setw(5) << setfill('0') << k;
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

        while(getline(measurement_file,point_string)){
            if (point_string.find("point") == 0){
                point_stream.str(point_string);
                point_stream >> dummy_string >> pt_idx >> landmark_id >> img_pt(0) >> img_pt(1);
                meas.detected_landmarks.push_back(landmark_id);
                meas.landmarks_img_pts.push_back(img_pt);
                point_stream.clear();
            }
        }

        measurements.push_back(meas);
        measurement_file.close();
        ss.str("");
    }    
}