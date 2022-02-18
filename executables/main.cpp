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
    Vector3fVector gt_trajectory;
    MeasVector measurements;
    Vector3fVector landmarks_gt;

    Vector3fVector landmarks; 

    FloatVector x_traj, y_traj, z_traj;
    FloatVector x_traj_gt, y_traj_gt, z_traj_gt;
    FloatVector x_l_gt, y_l_gt, z_l_gt;
    FloatVector x_l, y_l, z_l;

    pms_solver solver;

    int n_iterations = 30;

    load_trajectory(odom_trajectory,gt_trajectory);     
    
    prepare_for_plotting(odom_trajectory,x_traj,y_traj,z_traj);
    prepare_for_plotting(gt_trajectory,x_traj_gt,y_traj_gt,z_traj_gt);

    cout << "Showing the trajectory recorded by odometry:\n";
    gp << "set term qt 0" << endl;
    gp << "plot '-' using 1:2 with lines lt rgb 'blue' title 'Odometry Trajectory'," 
        << "'-' using 1:2 with points lt rgb 'red' title 'Groundtruth Trajectory'" << endl; 
    gp.send1d(boost::make_tuple(x_traj, y_traj, z_traj));
    gp.send1d(boost::make_tuple(x_traj_gt, y_traj_gt, z_traj_gt));
        
    cin.get();

    n_of_landmarks = load_measurements(measurements);

    landmarks_gt = load_landmarks_gt();

    landmarks = initial_guess(measurements,n_of_landmarks + 1);

    prepare_for_plotting(landmarks_gt,x_l_gt,y_l_gt,z_l_gt);
    prepare_for_plotting(landmarks,x_l,y_l,z_l);
    
    cout << "Showing the landmarks groundtruth and the initial guess by triangulation:\n";
    gp << "set term qt 1" << endl;
    gp << "splot '-' using 1:2:3 with points lt rgb 'red' title 'Landmarks GroundTruth',"
        << "'-' using 1:2:3 with points lt rgb 'blue' title 'Landmarks Initial Guess'" << endl; 
    gp.send1d(boost::make_tuple(x_l_gt, y_l_gt, z_l_gt));
    gp.send1d(boost::make_tuple(x_l, y_l, z_l));
    cin.get();

    solver.init(odom_trajectory,landmarks,measurements);
    solver.setProjKernelThreshold(1000.0f);
    solver.setOdomKernelThreshold(0.1f);

    cout << "Optimization of landmarks and trajectory:\n";    
    solver.least_square(n_iterations);
    cout << "Plotting the results:\n";

    prepare_for_plotting(odom_trajectory,x_traj,y_traj,z_traj);

    gp << "set term qt 0" << endl;
    gp << "plot '-' using 1:2 with lines lt rgb 'blue' title 'Odometry Trajectory'," 
        << "'-' using 1:2 with points lt rgb 'red' title 'Groundtruth Trajectory'" << endl; 
    gp.send1d(boost::make_tuple(x_traj, y_traj, z_traj));
    gp.send1d(boost::make_tuple(x_traj_gt, y_traj_gt, z_traj_gt));
        
    cin.get();

    prepare_for_plotting(landmarks_gt,x_l_gt,y_l_gt,z_l_gt);
    prepare_for_plotting(landmarks,x_l,y_l,z_l);
    
    gp << "set term qt 1" << endl;
    gp << "splot '-' using 1:2:3 with points lt rgb 'red' title 'Landmarks GroundTruth',"
        << "'-' using 1:2:3 with points lt rgb 'blue' title 'Landmarks Initial Guess'" << endl; 
    gp.send1d(boost::make_tuple(x_l_gt, y_l_gt, z_l_gt));
    gp.send1d(boost::make_tuple(x_l, y_l, z_l));
    cin.get();
    
    return 0;
}