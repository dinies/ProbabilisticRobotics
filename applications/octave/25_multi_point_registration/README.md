# 3D pose optimization with qualified landmarks

For testing (from **octave GUI command line**):

    $octave> MultiICP
    $octave> plotState(XL_true, XL_guess);
    $octave> [XR_guess, XL_guess, chi_stats, num_inliers] = doMultiICP(XR_guess, XL_guess, Z, associations, num_poses, num_landmarks, 10, 0, 1);
    $octave> chi_stats
    $octave> plotState(XL_true, XL_guess);

This shows the chi stat that goes down, and that the landmarks in 3d space are aligned.
More details in the file itself
