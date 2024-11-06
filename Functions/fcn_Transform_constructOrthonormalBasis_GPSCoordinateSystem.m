function [g_x_unit_gps,g_y_unit_gps,g_z_unit_gps] = fcn_Transform_constructOrthonormalBasis_GPSCoordinateSystem(GPS_SparkFun_Front_ENU,GPS_SparkFun_LeftRear_ENU,GPS_SparkFun_RightRear_ENU,varargin)
% fcn_Transform_constructOrthonormalBasis_GPSCoordinateSystem constructs the 
% Orthonormal Basis of rear right GPS coordiante system

%
% FORMAT:
%
% [g_x_unit_gps,g_y_unit_gps,g_z_unit_gps] = fcn_Transform_constructOrthonormalBasis_GPSCoordinateSystem(GPS_SparkFun_Front_ENU,GPS_SparkFun_LeftRear_ENU,GPS_SparkFun_RightRear_ENU,(flag_do_calibration),(fid),(fig_num))
%
% INPUTS:
%
%       GPS_SparkFun_Front_ENU: a Nx3 vectors of points containing the
%       interpolated ENU coordinates of the Front SparkFun GPS
%
%       GPS_SparkFun_LeftRear_ENU:  a Nx3 vectors of points containing the
%       interpolated ENU coordinates of the Rear Left SparkFun GPS
%
%       GPS_SparkFun_RightRear_ENU: a Nx3 vectors of points containing the
%       interpolated ENU coordinates of the Rear Right SparkFun GPS
%
%       (OPTIONAL INPUTS)
%
%       fid: a file ID to print results of analysis. If not entered, the
%       console (FID = 1) is used.
%
%       flag_do_calibration: a flag indicates whether the function is used
%       for calibration, if true, the output will be arrays of unit
%       vectors, otherwise, the output will be unit vectors along X, Y and
%       Z axes in rear right GPS coordinate system. If not entered,
%       flag_do_calibration = 0 is used.
%
%
% OUTPUTS:
%
%      g_x_unit_gps: the unit vector along x-axis
%
%      g_y_unit_gps: the unit vector along y-axis
%
%      g_z_unit_gps: the unit vector along z-axis
%
% DEPENDENCIES:
%
%      normalizeVector
%
% EXAMPLES:
%
%     See the script: script_test_fcn_Transform_Basis_GPSCoordinateSystem
%     for a full test suite.
%
% This function was written on 2023_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2023_10_20 - wrote the code
% 2024_01_28 - added more comments, particularly to explain inputs more
% clearly
% 2024_10_07 - xfc5113@psu.edu
% -- updated the input check area
% -- change flag_do_calibration from input to optional input
% -- add fig_num to optional input

flag_do_debug = 0;
flag_check_inputs = 1;
if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    % fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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

if flag_check_inputs
    % Are there the right number of inputs?
    narginchk(3,6);
end


% Does user spicift flag_do_calibration?
flag_do_calibration = 0;
if 4 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        flag_do_calibration = temp;
    end
end


% Does user want to specify fid?
fid = 0;
if 5 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        fid = temp;
    end
end

% Does user want to specify fig_num?
fig_num = -1;
flag_do_plots = 0;

if 6 <= nargin
    temp = varargin{3};
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

%% Step 1: Use the position of the rear right GPS antenna as the origin of the GPS coordiante system
origin_GPS_cooridnate_system = GPS_SparkFun_RightRear_ENU;
N_points = size(origin_GPS_cooridnate_system,1);
%% Step 2: Calculate two vectors point from the origin to the left and front GPS antenna
V_origin_left = GPS_SparkFun_LeftRear_ENU - origin_GPS_cooridnate_system; % a vector point from the origin to the left GPS antenna
V_origin_front = GPS_SparkFun_Front_ENU - origin_GPS_cooridnate_system; % a vector point from the origin to the front GPS antenna

%% Step 3: Compute vectors that form the basis
% V_origin_left identify the direction of Y-axis, normalize the vector to
% get the unit vector
g_y_unit = normalizeVector(V_origin_left); 
% g_y_unit_2 = normalize(V_origin_left,'norm'); 
% Compute a vector that normal to the GPS base plane with cross product
g_z = cross(V_origin_front,V_origin_left,2); 
% Normalized the vector, which is the second vector of the basis that identify the direction of Z-axis
g_z_unit = normalizeVector(g_z); 
% Use the cross product to compute the third vector of the basis, which identify the direction of X-axis
g_x_unit = cross(g_y_unit,g_z_unit,2); 

% If flag_do_calibration is true, we want to remove outliers first
if (N_points>1)&&(flag_do_calibration==1)
    [~,~,tfoutliers_x] = rmoutliers(g_x_unit);
    [~,~,tfoutliers_y] = rmoutliers(g_y_unit);
    [~,~,tfoutliers_z] = rmoutliers(g_z_unit);
    tfoutliers_xyz = tfoutliers_x|tfoutliers_y|tfoutliers_z;
    tfoutliers = any(tfoutliers_xyz,2);
    idxs_inliers = find(~tfoutliers);

    g_x_unit_gps = g_x_unit(idxs_inliers,:);
    g_y_unit_gps = g_y_unit(idxs_inliers,:);
    g_z_unit_gps = g_z_unit(idxs_inliers,:);


else
    g_x_unit_gps = g_x_unit;
    g_y_unit_gps = g_y_unit;
    g_z_unit_gps = g_z_unit;
end

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

%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

function V_unit = normalizeVector(V)
    V_mag = vecnorm(V,2,2);
    V_unit = V./V_mag;
end