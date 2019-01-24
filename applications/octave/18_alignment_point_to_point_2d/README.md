## `2d point-to-point ICP Optimization` (Octave)

While in this folder, run octave

     octave-cli

Source the file dontaining the needed methods

     source icp_2d.m
 

launch the program by choosing number of point, target transform and least squares iterations as:
 
     num_points = 5;
     target_transform = [20 -10 pi/3]';
     iterations = 10;
     [chi_ev, x_ev]=testICP2D(num_points, target_transform, iterations)

You will see the __chi_ev__ evolution (constantly decreasing), and the
solution found __x_ev__ that matches your target tranform
