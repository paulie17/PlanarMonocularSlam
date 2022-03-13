#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>

#include "defs.h"

#include "gnuplot-iostream.h"
#include "dataset_utils.h"
#include "pms_solver.h"

using namespace std;
using namespace Eigen;
using namespace pms;

int main(){

    int n_of_landmarks;
    int n_measurements;

    Gnuplot gp;
    
    Vector3fVector odom_trajectory;
    Vector3fVector gt_trajectory;
    MeasVector measurements;
    Vector3fVector landmarks_gt;

    Vector3fVector landmarks; 

    FloatVector x_traj, y_traj;
    FloatVector x_traj_gt, y_traj_gt;
    FloatVector x_l_gt, y_l_gt, z_l_gt;
    FloatVector x_l, y_l, z_l;
    IntVector discarded_landmarks;

    pms_solver solver;

    int n_iterations = 50;

    load_trajectory(odom_trajectory,gt_trajectory); 
    n_measurements = odom_trajectory.size();

    prepare_for_plotting(odom_trajectory,x_traj,y_traj);
    prepare_for_plotting(gt_trajectory,x_traj_gt,y_traj_gt);

    cout << "Showing the trajectory recorded by odometry together with the groundtruth trajectory.\n";
    gp << "set term qt 0" << endl;
    gp << "plot '-' using 1:2 with lines lt rgb 'red' title 'Groundtruth Trajectory'," 
        << "'-' using 1:2 with lines lt rgb 'blue' title 'Odometry Trajectory'" << endl; 
    gp.send1d(boost::make_tuple(x_traj_gt, y_traj_gt));
    gp.send1d(boost::make_tuple(x_traj, y_traj));            
    cout << "Press any key to continue. \n";
    cin.get();    
    
    load_measurements(measurements,n_measurements);

    cout << "Measurements loaded successfully! \n";

    landmarks_gt = load_landmarks_gt();

    n_of_landmarks = data_association(measurements);
    
    landmarks = initial_guess(measurements,n_of_landmarks,discarded_landmarks);
    cout << n_of_landmarks << " have been detected, " << discarded_landmarks.size() << " have been discarded for lack of enough measurements necessary for tringulation. \n" ; 

    prepare_for_plotting(landmarks_gt,x_l_gt,y_l_gt,z_l_gt);
    prepare_for_plotting(landmarks,x_l,y_l,z_l);
    
    cout << "Showing the landmarks groundtruth and the initial guess by triangulation.\n";
    gp << "set term qt 1" << endl;
    gp << "splot '-' using 1:2:3 with points lt rgb 'red' title 'Landmarks GroundTruth',"
        << "'-' using 1:2:3 with points lt rgb 'blue' title 'Landmarks Initial Guess'" << endl; 
    gp.send1d(boost::make_tuple(x_l_gt, y_l_gt, z_l_gt));
    gp.send1d(boost::make_tuple(x_l, y_l, z_l));
    cout << "Press any key to continue. \n";
    cin.get();
    
    solver.init(odom_trajectory,landmarks,measurements);
    solver.setProjKernelThreshold(1000.0f);
    solver.setOdomKernelThreshold(0.001f);

    cout << "Optimization of landmarks and trajectory:\n";   

    for (int i=0;i<n_iterations;i++){
        solver.one_round();
        cout << "**********************************" << endl;
        cout << "Iteration " << i + 1 << endl;
        cout << "Chi odometry: " << solver.chiOdom() << endl ;
        cout << "Chi projections: " << solver.chiProj() << endl; 
        cout << "**********************************" << endl;
    }

    cout << "Plotting the results:\n";

    prepare_for_plotting(odom_trajectory,x_traj,y_traj);

    gp << "set term qt 0" << endl;
    gp << "plot '-' using 1:2 with lines lt rgb 'red' title 'Groundtruth Trajectory'," 
        << "'-' using 1:2 with lines lt rgb 'blue' title 'Odometry Trajectory after LS Optimization'" << endl; 
    gp.send1d(boost::make_tuple(x_traj_gt, y_traj_gt));
    gp.send1d(boost::make_tuple(x_traj, y_traj));        
    cout << "Press any key to continue. \n";
    cin.get();

    prepare_for_plotting(landmarks,x_l,y_l,z_l);
    
    gp << "set term qt 1" << endl;
    gp << "splot '-' using 1:2:3 with points lt rgb 'red' title 'Landmarks GroundTruth',"
        << "'-' using 1:2:3 with points lt rgb 'blue' title 'Landmarks after LS Optimization'" << endl; 
    gp.send1d(boost::make_tuple(x_l_gt, y_l_gt, z_l_gt));
    gp.send1d(boost::make_tuple(x_l, y_l, z_l));
    cout << "Press any key to continue. \n";
    cin.get();

    return 0;
}