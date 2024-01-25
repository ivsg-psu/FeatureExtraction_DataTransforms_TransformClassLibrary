function M = fcn_Transformation_FitTransformationSVD(inputPoints,targetPoints,W_array)
% fcn_Transformation_FitTransformationSVD computing the best-fitting rigid 
% transformation that aligns two sets of corresponding points.
% FORMAT:
%
% M = fcn_Transformation_FitTransformationSVD(inputPoints, targetPoints,W_array)
%
% INPUTS:
%
%      inputPoints: a Nx3 vectors containing xyz coordiantes of the
%      original points
%      
%      targetPoints: a Nx3 vectors containing xyz coordiantes of the target
%      points
%       
%      W_array：a Nx1 vector containing the weighting factors for all the
%      points
%
%
% OUTPUTS:
%
%      M: a homogeneous transformation matrix
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:   
%
% See the script: script_test_fcn_Transformation_FitTransformationSVD
% for a full test suite.
%
% This function was written on 2024_01_24 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2024_01_24 - xfc5113@psu.edu
% -- original write of the code

%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.

if (nargin==2)
    flag_do_debug = 0; % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_GEOMETRY_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_GEOMETRY_FLAG_CHECK_INPUTS");
    MATLABFLAG_GEOMETRY_FLAG_DO_DEBUG = getenv("MATLABFLAG_GEOMETRY_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_GEOMETRY_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_GEOMETRY_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_GEOMETRY_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_GEOMETRY_FLAG_CHECK_INPUTS);
    end
end
if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end

%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _       
%  |_   _|                 | |      
%    | |  _ __  _ __  _   _| |_ ___ 
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |                  
%              |_| 
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(3,3);

        % Check the inputPoints input
        fcn_DebugTools_checkInputsToFunctions(...
            inputPoints, '3column_of_numbers');
        fcn_DebugTools_checkInputsToFunctions(...
            targetPoints, '3column_of_numbers');
       

end


%% Solve for the circle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the weighted centroids of both point sets
inputPoints_center  = sum(W_array.*inputPoints,1)/sum(W_array);
targetPoints_center = sum(W_array.*targetPoints,1)/sum(W_array);
% Compute the centered vectors
inputPoints_centered = inputPoints - inputPoints_center;
targetPoints_centered = targetPoints - targetPoints_center;
W_matrix = diag(W_array);
% Compute the d × d covariance matrix
P = inputPoints_centered.'*W_matrix*targetPoints_centered;
% Compute the singular value decomposition 
% Compute the rotation matrix
[U,~,V] = svd(P);
R = V*U.';
F = eye(3);
F(3,3) = det(R);
R = V*F*U.';
% Compute the optimal translation
t = targetPoints_center.' - R*inputPoints_center.';
% Construct the homogeneous transformation matrix
M = [R,t;0 0 0 1];
disp('Done Motion Fitting')
end
