script_test_fcn_Transform_FitTransformationSVD

%% Test case 1: Translation Only
inputPoints_case_1 = [2 -2 4;
                      3 5 0;
                      1 4 3];
T_case_1 = makehgtform('translate',[1 1 1]);
N_points_case_1 = size(inputPoints_case_1,1);
targetPoints_case_1 = fcn_Internal_calculateTargetPoints(inputPoints_case_1, T_case_1);

W_array = ones(N_points_case_1,1);
T_case_1_fitted = fcn_Transformation_FitTransformationSVD(inputPoints_case_1, targetPoints_case_1,W_array);

%% Test case 2: Rotation along X-axis Only
inputPoints_case_2 = [2 -2 4;
                      3 5 0;
                      1 4 3;
                      5 0 2];
N_points_case_2 = size(inputPoints_case_2,1);
T_case_2 = makehgtform('xrotate',pi/2);
targetPoints_case_2 = fcn_Internal_calculateTargetPoints(inputPoints_case_2, T_case_2);
W_array = ones(N_points_case_2,1);
T_case_2_fitted = fcn_Transformation_FitTransformationSVD(inputPoints_case_2, targetPoints_case_2,W_array);

%% Test case 3: Rotation along Y-axis Only
inputPoints_case_3 = [2 -2 4;
                      3 5 0;
                      1 4 3;
                      5 0 2];
N_points_case_3 = size(inputPoints_case_3,1);
T_case_3 = makehgtform('yrotate',pi/4);
targetPoints_case_3 = fcn_Internal_calculateTargetPoints(inputPoints_case_3, T_case_2);
W_array = ones(N_points_case_3,1);
T_case_3_fitted = fcn_Transformation_FitTransformationSVD(inputPoints_case_3, targetPoints_case_3 ,W_array);

%% Test case 4: Rotation along Z-axis Only
inputPoints_case_4 = [2 -2 4;
                      3 5 0;
                      1 4 3;
                      5 0 2];
N_points_case_4 = size(inputPoints_case_4,1);
T_case_4 = makehgtform('yrotate',-pi/3);
targetPoints_case_4 = fcn_Internal_calculateTargetPoints(inputPoints_case_4, T_case_4);
W_array = ones(N_points_case_4,1);
T_case_4_fitted = fcn_Transformation_FitTransformationSVD(inputPoints_case_4, targetPoints_case_4 ,W_array);

%% Test case 5: Translation and Rotation
inputPoints_case_5 = [2 -2 4;
                      3 5 0;
                      1 4 3;
                      5 0 2];
N_points_case_5 = size(inputPoints_case_5,1);
T_case_5 = T_case_1*T_case_4*T_case_3*T_case_2;
targetPoints_case_5 = fcn_Internal_calculateTargetPoints(inputPoints_case_5, T_case_5);
W_array = ones(N_points_case_5,1);
T_case_5_fitted = fcn_Transformation_FitTransformationSVD(inputPoints_case_5, targetPoints_case_5 ,W_array);
%% Test case 6: Translation and Rotation but with outliers
inputPoints_case_6 = [2 -2 4;
                      3 5 0;
                      1 4 3;
                      5 0 2;
                      4 2 7];
N_points_case_6 = size(inputPoints_case_6,1);
T_case_6 = T_case_5;
targetPoints_case_6 = fcn_Internal_calculateTargetPoints(inputPoints_case_6, T_case_6);
% replace the last point with an outlier
targetPoints_case_6(end,:) = [1 1 0];
W_array = ones(N_points_case_6,1);
T_case_6_fitted = fcn_Transformation_FitTransformationSVD(inputPoints_case_6, targetPoints_case_6 ,W_array);
transformedPoints_case_6 = fcn_Internal_calculateTargetPoints(inputPoints_case_6, T_case_6);
[dist_pts, fitting_error] = fcn_Internal_calculateFittingError(targetPoints_case_6, transformedPoints_case_6);
%% Functions
function targetPoints = fcn_Internal_calculateTargetPoints(inputPoints, T_transform)
    N_points = size(inputPoints,1);
    inputPoints_homo = [inputPoints, ones(N_points,1)];
    targetPoints_homo = T_transform*inputPoints_homo.';
    targetPoints_transpose = targetPoints_homo(1:3,:);
    targetPoints = targetPoints_transpose.';
end

function [dist_pts, fitting_error] = fcn_Internal_calculateFittingError(targetPoints, transformedPoints)
    dist_pts = vecnorm(transformedPoints - targetPoints,2,2);
    fitting_error = mean(dist_pts);
end