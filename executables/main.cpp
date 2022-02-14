#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>

#include "defs.h"

#include "gnuplot-iostream.h"
#include "dataset_utils.h"
#include "bootstrap.h"
#include "pms_solver.h"

using namespace std;
using namespace Eigen;
using namespace pr;

int main(){

    int n_of_landmarks;

    Gnuplot gp;
    Vector3fVector odom_trajectory;
    MeasVector measurements;
    Vector3fVector landmarks_gt;

    Vector3fVector landmarks; 

    //vector<double> x_traj, y_traj, z_traj;
    vector<double> x_l_gt, y_l_gt, z_l_gt;
    vector<double> x_l, y_l, z_l;

    pms_solver solver;

    int n_iterations = 10;

    odom_trajectory = load_trajectory();     
    
    /*for (int i = 0; i < NUM_MEASUREMENTS; i++){
        x_traj.push_back(odom_trajectory[i](0));
        y_traj.push_back(odom_trajectory[i](1));
        z_traj.push_back(odom_trajectory[i](2));
    }*/

    //gp << "set term qt 0" << endl;
    //gp << "splot '-' using 1:2:3 with lines lt rgb 'red' title 'Trajectory'" << endl;
    //gp.send1d(boost::make_tuple(x_traj, y_traj, z_traj));
        
    //cin.get();

    n_of_landmarks = load_measurements(measurements);

    landmarks_gt = load_landmarks_gt();

    for (int i = 0; i < (n_of_landmarks + 1); i++){
        x_l_gt.push_back(landmarks_gt[i](0));
        y_l_gt.push_back(landmarks_gt[i](1));
        z_l_gt.push_back(landmarks_gt[i](2));
    }

    //gp << "splot '-' using 1:2:3 with lines lt rgb 'red' title 'Trajectory',"
    //    << "'-' using 1:2:3 with points lt rgb 'red' title 'Landmarks GroundTruth'" << endl;

    //gp << "set term qt 1" << endl;
    //gp << "splot '-' using 1:2:3 with points lt rgb 'red' title 'Landmarks GroundTruth'" << endl;
    //gp.send1d(boost::make_tuple(x_traj, y_traj, z_traj));
    //gp.send1d(boost::make_tuple(x_l_gt, y_l_gt, z_l_gt));

    //cin.get();

    landmarks = initial_guess(measurements,n_of_landmarks + 1);

    for (int i = 0; i < (n_of_landmarks + 1); i++){
        x_l.push_back(landmarks[i](0));
        y_l.push_back(landmarks[i](1));
        z_l.push_back(landmarks[i](2));
    }
    
    /*gp << "splot '-' using 1:2:3 with points lt rgb 'red' title 'Landmarks GroundTruth',"
        << "'-' using 1:2:3 with points lt rgb 'blue' title 'Landmarks Initial Guess'" << endl; 
    gp.send1d(boost::make_tuple(x_l_gt, y_l_gt, z_l_gt));
    gp.send1d(boost::make_tuple(x_l, y_l, z_l));
    cin.get();*/

    solver.init(odom_trajectory,landmarks,measurements);

    for (int i = 0; i < n_iterations; i++){
        solver.one_round();
    }    

    return 0;
}