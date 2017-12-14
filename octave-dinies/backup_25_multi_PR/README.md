# simple example of 3D pose optimization with qualified landmarks
to test it:

- run octave

        MultiICP
        plotState(XL_true, XL_guess);
        [XR_guess, XL_guess, chi_stats, num_inliers] = doMultiICP(XR_guess, XL_guess, Z, associations, num_poses, num_landmarks, 10, 0, 1);
        chi_stats
        plotState(XL_true, XL_guess);

This shows the `chi_stat` that goes down, and that the landmarks in 3d space are aligned.
More details in the file itself
