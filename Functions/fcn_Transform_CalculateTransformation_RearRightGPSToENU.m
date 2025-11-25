function M_transform_RearRightGPS_to_ENU = fcn_Transform_calculateTransformation_RearRightGPSToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU, varargin)

% fcn_Transform_CalculateTransformation_RearRightGPSToENU
% This function takes three GPS Antenna centers, GPSFront_ENU, GPSLeft_ENU and 
% GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
% [x, y, z] in meters as the inputs and outputs the transformation matrix
% from rear right GPS coordiante system to ENU coordinate system
%
%
% FORMAT:
%
%      M_transform_RearRightGPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU, (fid), (fig_num))
%
% INPUTS:
%
%      GPSFront_ENU: a 1x3 array contains the ENU coordinates for front GPS
%
%      GPSLeft_ENU: a 1x3 array contains the ENU coordinates for rear left GPS
%
%      GPSRight_ENU: a 1x3 array contains the ENU coordinates for rear right GPS
%
%
%      fid: the fileID where to print. Default is 1, to print results to
%      the console.
%
%      fig_num: a scalar integer value
%
%
% OUTPUTS:
%
%      M_transform_RearRightGPS_to_ENU: a transformation matrix from rear
%      right GPS coordiante system to ENU coordinate system
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
%     See the script: script_test_fcn_Transform_RearRightGPSToENU
%     for a full test suite.
%
% This function was written on 2024_11_02 by X. Cao
% Questions or comments? xfc5113@psu.edu

% Revision history
% 2024_11_02 - Xinyu Cao, xfc5113@psu.edu
% -- wrote the code originally


% To do list:
% Edit the comments
% Add comments to some new created functions

%% Debugging and Input checks

flag_do_debug = 0; % % % % Flag to plot the results for debugging
flag_check_inputs = 1; % Flag to perform input checking

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
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


if flag_check_inputs == 1
    % Are there the right number of inputs?
    narginchk(3,5);
end


% Does user want to specify fid?
fid = 0;
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        fid = temp;
    end
end

% Does user want to specify fig_num?
fig_num = -1;
flag_do_plots = 0;

if 5 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        fig_num = temp;
        flag_do_plots = 1;
    end
end

   

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end



%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step 1 - Finding the Roll, Pitch and Yaw angle of the vehicle relative to ENU coordinates
[g_x_unit_gps,g_y_unit_gps,g_z_unit_gps] = fcn_Transform_constructOrthonormalBasis_GPSCoordinateSystem(GPSFront_ENU,GPSLeft_ENU,GPSRight_ENU);
%% Construct the orthonormal basis for the rear right GPS coordinate system
Rotation_GPS_to_ENU = [g_x_unit_gps.' g_y_unit_gps.' g_z_unit_gps.'];

%% Step 2 - Find the Position of the Sensor Mount and find the rotation matrix from rear right GPS coordiante system to ENU coordinate system

origin_GPS_cooridnate_system = GPSRight_ENU;
translation_GPS_to_ENU = origin_GPS_cooridnate_system;

%% Step 3 - Calculate transformation from rear right GPS to ENU coordinate

M_transform_RearRightGPS_to_ENU = se3(Rotation_GPS_to_ENU, translation_GPS_to_ENU);

%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_plots
    figure(fig_num)

    % Plot the Left GPS center
    quiver3(origin_GPS_cooridnate_system(1,1), origin_GPS_cooridnate_system(1,2), origin_GPS_cooridnate_system(1,3), g_x_unit_gps(1,1), g_x_unit_gps(1,2), g_x_unit_gps(1,3), 'Color', 'b', 'LineWidth', 1);
    hold on
    quiver3(origin_GPS_cooridnate_system(1,1), origin_GPS_cooridnate_system(1,2), origin_GPS_cooridnate_system(1,3), g_y_unit_gps(1,1), g_y_unit_gps(1,2), g_y_unit_gps(1,3), 'Color', 'r', 'LineWidth', 1);
    quiver3(origin_GPS_cooridnate_system(1,1), origin_GPS_cooridnate_system(1,2), origin_GPS_cooridnate_system(1,3), g_z_unit_gps(1,1), g_z_unit_gps(1,2), g_z_unit_gps(1,3), 'Color', 'g', 'LineWidth', 1);
    xlabel('X East [m]')
    ylabel('Y North [m]')
    zlabel('Z Up [m]')
    axis equal

    angle_XY = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(g_x_unit_gps,g_y_unit_gps));
    angle_XZ = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(g_x_unit_gps,g_z_unit_gps));
    angle_YZ = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(g_y_unit_gps,g_z_unit_gps));
    fprintf("Angle between X and Y axis is %d degree\n\n", angle_XY)
    fprintf("Angle between X and Z axis is %d degree\n\n", angle_XZ)
    fprintf("Angle between Y and Z axis is %d degree\n\n", angle_YZ)
end

if flag_do_debug
    fprintf(fileID,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end