% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2566.626740722335398 ; 2576.240705615100069 ];

%-- Principal point:
cc = [ 1250.553746569204804 ; 1026.111454605519839 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.040743976238054 ; -1.323466673420614 ; -0.001999651410394 ; -0.008939826355333 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 23.469951352014462 ; 22.685389716506283 ];

%-- Principal point uncertainty:
cc_error = [ 35.607295001861438 ; 31.613726037050569 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.047201374301537 ; 0.398840524102621 ; 0.003864165002168 ; 0.004411223377647 ; 0.000000000000000 ];

%-- Image size:
nx = 1944;
ny = 2592;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 15;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.881910e+00 ; -2.301989e+00 ; 1.743285e-01 ];
Tc_1  = [ -1.754637e+02 ; -1.703216e+01 ; 6.302455e+02 ];
omc_error_1 = [ 1.210331e-02 ; 1.471211e-02 ; 2.567740e-02 ];
Tc_error_1  = [ 8.798597e+00 ; 7.884108e+00 ; 5.979783e+00 ];

%-- Image #2:
omc_2 = [ -1.557398e+00 ; -2.681906e+00 ; -7.915632e-03 ];
Tc_2  = [ -1.231520e+02 ; -6.174152e+01 ; 6.419257e+02 ];
omc_error_2 = [ 1.071421e-02 ; 1.896704e-02 ; 3.039545e-02 ];
Tc_error_2  = [ 8.891000e+00 ; 7.944680e+00 ; 6.367506e+00 ];

%-- Image #3:
omc_3 = [ 1.676111e+00 ; 2.076200e+00 ; 6.299848e-01 ];
Tc_3  = [ -1.318829e+02 ; -8.859239e+00 ; 5.744179e+02 ];
omc_error_3 = [ 1.235992e-02 ; 9.814100e-03 ; 1.957547e-02 ];
Tc_error_3  = [ 7.955370e+00 ; 7.119836e+00 ; 6.099798e+00 ];

%-- Image #4:
omc_4 = [ -1.767612e+00 ; -1.882158e+00 ; 4.712769e-01 ];
Tc_4  = [ -2.180136e+02 ; 1.682974e+01 ; 7.095583e+02 ];
omc_error_4 = [ 1.235328e-02 ; 1.237318e-02 ; 1.921749e-02 ];
Tc_error_4  = [ 1.005967e+01 ; 8.908078e+00 ; 6.434119e+00 ];

%-- Image #5:
omc_5 = [ -1.245385e+00 ; -2.608638e+00 ; 3.184601e-01 ];
Tc_5  = [ -1.141900e+02 ; -5.233026e+01 ; 6.660415e+02 ];
omc_error_5 = [ 8.690112e-03 ; 1.578883e-02 ; 2.159443e-02 ];
Tc_error_5  = [ 9.227511e+00 ; 8.223958e+00 ; 5.909469e+00 ];

%-- Image #6:
omc_6 = [ 1.433320e+00 ; 1.869820e+00 ; 4.018550e-01 ];
Tc_6  = [ -3.259282e+01 ; -1.160568e+02 ; 4.721210e+02 ];
omc_error_6 = [ 1.065685e-02 ; 1.183288e-02 ; 1.605129e-02 ];
Tc_error_6  = [ 6.582840e+00 ; 5.795268e+00 ; 4.649682e+00 ];

%-- Image #7:
omc_7 = [ -1.552174e+00 ; -1.773400e+00 ; 5.589938e-01 ];
Tc_7  = [ -8.460894e+01 ; -1.127405e+02 ; 5.955305e+02 ];
omc_error_7 = [ 1.158396e-02 ; 1.015317e-02 ; 1.513557e-02 ];
Tc_error_7  = [ 8.266830e+00 ; 7.318201e+00 ; 4.977908e+00 ];

%-- Image #8:
omc_8 = [ -1.620656e+00 ; -1.737020e+00 ; -2.690329e-01 ];
Tc_8  = [ -9.210330e+01 ; -6.755549e+01 ; 6.086703e+02 ];
omc_error_8 = [ 8.944702e-03 ; 1.207503e-02 ; 1.606029e-02 ];
Tc_error_8  = [ 8.430801e+00 ; 7.482766e+00 ; 5.718268e+00 ];

%-- Image #9:
omc_9 = [ NaN ; NaN ; NaN ];
Tc_9  = [ NaN ; NaN ; NaN ];
omc_error_9 = [ NaN ; NaN ; NaN ];
Tc_error_9  = [ NaN ; NaN ; NaN ];

%-- Image #10:
omc_10 = [ 2.097749e+00 ; 1.671917e+00 ; 9.271959e-01 ];
Tc_10  = [ -8.118671e+01 ; -6.398190e+01 ; 4.317302e+02 ];
omc_error_10 = [ 1.396085e-02 ; 8.105942e-03 ; 1.876422e-02 ];
Tc_error_10  = [ 6.026459e+00 ; 5.292780e+00 ; 4.567462e+00 ];

%-- Image #11:
omc_11 = [ 2.079116e+00 ; 1.298406e+00 ; 1.203964e+00 ];
Tc_11  = [ -7.668320e+01 ; -2.787674e+01 ; 4.394898e+02 ];
omc_error_11 = [ 1.496207e-02 ; 7.869266e-03 ; 1.725969e-02 ];
Tc_error_11  = [ 6.107821e+00 ; 5.395087e+00 ; 4.786855e+00 ];

%-- Image #12:
omc_12 = [ NaN ; NaN ; NaN ];
Tc_12  = [ NaN ; NaN ; NaN ];
omc_error_12 = [ NaN ; NaN ; NaN ];
Tc_error_12  = [ NaN ; NaN ; NaN ];

%-- Image #13:
omc_13 = [ -2.147102e+00 ; -2.117337e+00 ; -9.097459e-02 ];
Tc_13  = [ -1.082246e+02 ; -7.827013e+01 ; 5.162640e+02 ];
omc_error_13 = [ 1.120827e-02 ; 1.167462e-02 ; 2.460493e-02 ];
Tc_error_13  = [ 7.195545e+00 ; 6.391153e+00 ; 5.171145e+00 ];

%-- Image #14:
omc_14 = [ 1.879512e+00 ; 1.869085e+00 ; -5.806875e-01 ];
Tc_14  = [ -1.081439e+02 ; -8.272629e+01 ; 5.621824e+02 ];
omc_error_14 = [ 8.480467e-03 ; 1.237782e-02 ; 1.840033e-02 ];
Tc_error_14  = [ 7.817642e+00 ; 6.922822e+00 ; 4.964080e+00 ];

%-- Image #15:
omc_15 = [ 2.052015e+00 ; 2.006626e+00 ; -3.961269e-01 ];
Tc_15  = [ -1.038231e+02 ; -5.666468e+01 ; 6.058787e+02 ];
omc_error_15 = [ 9.661661e-03 ; 1.192446e-02 ; 2.094931e-02 ];
Tc_error_15  = [ 8.391681e+00 ; 7.426195e+00 ; 5.396408e+00 ];

