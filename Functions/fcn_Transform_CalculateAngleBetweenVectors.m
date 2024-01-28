function [angle, angle_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_1, V_2, varargin)
% fcn_Transform_CalculateAngleBetweenVectors calculates the angle in rad
% between two vectors
% two vectors given as V_1 and V_2
%
% FORMAT:
%
% [angle, angle_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_1, V_2,(fig_num))
%
% INPUTS:
%
%      V_1: a 1x3 vector
%
%      V_2: a 1x3 vector
%
%      (OPTIONAL INPUTS)
%
%      fig_num: a figure number to plot results. If set to -1, skips any
%      input checking or debugging, no figures will be generated, and sets
%      up code to maximize speed.
%
% OUTPUTS:
%
%      angle: the angle between two vectors, as a scaler, from V_1 to V_2
%
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%      fcn_geometry_plotCircle
%
% EXAMPLES:
%      
%      % BASIC example
%      points = [0 0; 1 4; 0.5 -1];
%      [centers,radii] = fcn_geometry_circleCenterFrom3Points(points,1)
% 
% See the script: script_test_fcn_Transform_CalculateAngleBetweenVectors
% for a full test suite.
%
% This function was written on 2023_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2023_10_20 - wrote the code
% 2024_01_28 - added more comments, particularly to explain inputs more
% clearly

%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
flag_max_speed = 0;
if (nargin==4 && isequal(varargin{3},-1)) || (nargin==2 && isequal(varargin{1},-1))
    flag_do_debug = 0; % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
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


if flag_max_speed==0
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(1,4);

        % Check the points input
        fcn_DebugTools_checkInputsToFunctions(...
            points1, '2column_of_numbers');
    end
end

% Is the user giving separated input point vectors?
flag_use_separated_point_inputs = 0;
if nargin>2
    flag_use_separated_point_inputs = 1;
    temp = varargin{1};
    if ~isempty(temp)
        points2 = temp;
    else
        error('Expected 2nd input to be a point type')
    end

    if nargin>=3
        temp = varargin{2};
        if ~isempty(temp)
            points3 = temp;
        else
            error('Expected 3rd input to be a point type')
        end
    end

    N_points = length(points1(:,1));

    if flag_check_inputs
        % Check the points2 input
        fcn_DebugTools_checkInputsToFunctions(...
            points2, '2column_of_numbers',[N_points N_points]);

        % Check the points3 input
        fcn_DebugTools_checkInputsToFunctions(...
            points3, '2column_of_numbers',[N_points N_points]);
    end
end

% Does user want to show the plots?
flag_do_plots = 0;
if 0==flag_max_speed
    if (2 == nargin || 4 == nargin)
        temp = varargin{end};
        if ~isempty(temp)
            fig_num = temp;
            flag_do_plots = 1;
        end
    end
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

% Normalize the first vector, find the unit vector
V_1_mag = vecnorm(V_1,2,2);
V_1_unit = V_1./V_1_mag;
% Normalize the second vector, find the unit vector
V_2_mag = vecnorm(V_2,2,2);
V_2_unit = V_2./V_2_mag;
% Use the cross product to calculate the angle from V_1_unit to V_2_unit
angle = (asin(vecnorm(cross(V_1_unit,V_2_unit,2),2,2)));
angle_deg = rad2deg(angle);