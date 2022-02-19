# PlanarMonocularSlam

### Dependencies (Ubuntu 20.04 LTS or previous)

    CMake build utilities:

    sudo apt install build-essential cmake
    
    Eigen3: http://eigen.tuxfamily.org

    sudo apt install libeigen3-dev
    
    Gnuplot-Iostream Interface:

    sudo apt install libgnuplot-iostream-dev


### Compilation
From the system console, execute the build sequence (out of source build):

    mkdir build
    cd build
    cmake ..
    make
    
### Execution
The project provides the following binary in the `build/executables` folder:
- `./main`: Import dataset with landmarks projections and odometry; bootstrap an initial guess via triangulation; optimize landmarks and trajectory via least squares optimization. 
