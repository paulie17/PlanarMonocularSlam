#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include "defs.h"

#include "gnuplot-iostream.h"

using namespace std;
using namespace Eigen;
using namespace pr;

int main(){
    
    string dummy_string;
    int current_pose;

    Vector6fVector odom_trajectory;  
    Vector6f odom_pose;

    Gnuplot gp;

    MatrixXf matrix_for_plotting(200, 3);

    vector<double> x_hist, y_hist, z_hist;

    ifstream trajectory_file("../dataset/trajectory.dat");    

    while (getline(trajectory_file,dummy_string)){
        istringstream ss(dummy_string);
        ss >> dummy_string >> current_pose;
        
        for (int i = 0; i<6; i++){
            ss >> odom_pose(i);
        }        

        matrix_for_plotting.row(current_pose) = odom_pose.head(3);

        //cout << current_pose << endl;
        //cout << odom_pose << endl;
        odom_trajectory.push_back(odom_pose);
    }        

    for (int i = 0; i < 200; i++){
        x_hist.push_back(matrix_for_plotting(i,0));
        y_hist.push_back(matrix_for_plotting(i,1));
        z_hist.push_back(matrix_for_plotting(i,2));
    }

    gp << "splot '-' using 1:2:3 with lines lt rgb 'red' title 'Trajectory'" << endl;
    gp.send1d(boost::make_tuple(x_hist, y_hist, z_hist));
        
    //cout << x << endl;
    cin.get();
    //cout << x_hist[5] << endl;
    return 0;
}