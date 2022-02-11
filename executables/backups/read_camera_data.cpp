#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
//#include "defs.h"

using namespace std;
using namespace Eigen;

int main(){

    string dummy_string;

    Matrix3f cameraMatrix;
    Matrix4f camera_Transform;
    Vector3f t;
    Matrix3f R;
    Isometry3f camera_to_robot;
    int z_near,z_far;
    int width, height;
    //matrixA.setZero();
    //cout << matrixA <<endl;

    ifstream camera_file("../dataset/camera.dat");
    getline(camera_file, dummy_string);
    
    for (int i = 0; i < 3; i ++) {
        for (int j = 0; j<3; j++){
            camera_file >> cameraMatrix(i,j);
        }    
    }

    camera_file >> dummy_string;
    for (int i = 0; i < 4; i ++) {
        for (int j = 0; j<4; j++){
            camera_file >> camera_Transform(i,j);
        }    
    }

    camera_file >> dummy_string;
    camera_file >> dummy_string >> z_near;
    camera_file >> dummy_string >> z_far;
    camera_file >> dummy_string >> width;
    camera_file >> dummy_string >> height;
    
    t = camera_Transform.block(0,3,3,1);
    R = camera_Transform.block(0,0,3,3);

    camera_to_robot.setIdentity();
    camera_to_robot.translation() = t;
    camera_to_robot.prerotate(R);
    cout << camera_to_robot.matrix() << endl;

    return 0;
}