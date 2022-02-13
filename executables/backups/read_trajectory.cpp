#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include "defs.h"

using namespace std;
using namespace Eigen;
using namespace pr;

int main(){
    
    string dummy_string;
    int current_pose;
    Vector6fVector odom_trajectory;  
    Vector6f odom_pose;
    ifstream trajectory_file("../dataset/trajectory.dat");

    while (getline(trajectory_file,dummy_string)){
        std::istringstream ss(dummy_string);
        ss >> dummy_string >> current_pose;
        for (int i = 0; i<6; i++){
            ss >> odom_pose(i);
        }        
        cout << current_pose << endl;
        cout << odom_pose << endl;
        odom_trajectory.push_back(odom_pose);
    }
    //getline(trajectory_file,dummy_string);
    //trajectory_file >> dummy_string;
    //cout << dummy_string << endl;
    //cout << odom_trajectory[199] << endl;
}