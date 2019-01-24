    PICP: Projective ICP

### Dependencies (Ubuntu 16.04.3 LTS)
The following packages are required <br>
Before installing anything one should make sure what packages are already installed on the system!

CMake build utilities:

    sudo apt install build-essential cmake
    
OpenCV 2/3: https://opencv.org

    sudo apt install libopencv-dev
    
Eigen3: http://eigen.tuxfamily.org

    sudo apt install libeigen3-dev

### Compilation
From the system console, execute the build sequence (out of source build):

    mkdir build
    cd build
    cmake ..
    make
    
### Execution
The project provides the following 5 binaries in the `build/executables` folder:
- `./camera_test`: Testing of the pinhole camera projection function
- `./distance_map_test`: Testing of the 2d distance map computation (assuming projected points)
- `./correspondence_finder_test`: Unittest for the correspondence finding algorithm that uses the projection and the distance map functionality)
- `./picp_solver_test`: Least squares solver testing on artifical data and known correspondences
- `./picp_complete_test`: Complete PICP program
