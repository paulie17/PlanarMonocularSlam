# PlanarMonocularSlam

### Compilation
From the system console, execute the build sequence (out of source build):

    mkdir build
    cd build
    cmake ..
    make
    
### Execution
The project provides the following binary in the `build/executables` folder:
- `./main`: Import dataset with landmarks projections and odometry; bootstrap an initial guess via triangulation; optimize landmarks and trajectory via least squares optimization. 
