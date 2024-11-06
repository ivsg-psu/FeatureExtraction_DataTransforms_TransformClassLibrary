function LiDAR_PointCloud_Transformed_Cell = fcn_Transform_transformLiDARToENU(dataStructure,varargin)


% fcn_Transform_transformLiDARToENU
% This function takes three GPS Antenna centers, GPSFront_ENU, GPSLeft_ENU and 
% GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
% [x, y, z] in meters and LiDAR point cloud data as the inputs and outputs
% the LiDAR point cloud data in ENU coordinate system
%
%
% FORMAT:
%
%      LiDAR_PointCloudArray_ENU = fcn_Transform_CalculateTransformation_VehicleToENU(dataStructure, (fid), (fig_num))
%
% INPUTS:
%      
%      dataStructure: cleaned data structure
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
%      LiDAR_PointCloud_Transformed_Cell: A cell array contains LiDAR point 
%      cloud data in ENU coordinate system
%
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
%     See the script: script_test_fcn_Transform_transformLiDARToENU
%     for a full test suite.
%
% This function was written on 2024_11_05 by X. Cao
% Questions or comments? xfc5113@psu.edu

% Revision history
% 2024_11_05 - Xinyu Cao, xfc5113@psu.edu
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
    narginchk(1,3);
end

% % Does user want to specify calibration matrix?
% M_transform_LiDARVelodyne_to_RearRightGPS = load("Data\M_transform_LiDARVelodyne_to_RearRightGPS_2024_10_23.mat");
% if 2 <= nargin
%     temp = varargin{1};
%     if ~isempty(temp)
%         M_transform_LiDARVelodyne_to_RearRightGPS = temp;
%     end
% end

% Does user want to specify fid?
fid = 0;
if 2 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        fid = temp;
    end
end


% Does user want to specify fig_num?
fig_num = -1;
flag_do_plots = 0;
if 3 <= nargin
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


%% Step 1 - Grab ENU data from GPS units
[GPS_SparkFun_Front_ENU_array, GPS_SparkFun_LeftRear_ENU_array, GPS_SparkFun_RightRear_ENU_array] = fun_Transform_extractENUCoordinatesFromGPS(dataStructure);
%% Step 2 - Grab LiDAR point cloud data
LiDAR_Velodyne_Rear_structure = dataStructure.LiDAR_Velodyne_Rear;
LiDAR_PointCloud_Cell = LiDAR_Velodyne_Rear_structure.PointCloud;
%% Step 3 - Transform LiDAR point cloud data from LiDAR coordinate system to ENU coordinate system
N_scans = length(LiDAR_PointCloud_Cell);
LiDAR_PointCloud_Transformed_Cell = {};
for idx_scan = 1:N_scans
    GPSFront_ENU = GPS_SparkFun_Front_ENU_array(idx_scan,:);
    GPSLeft_ENU = GPS_SparkFun_LeftRear_ENU_array(idx_scan,:);
    GPSRight_ENU = GPS_SparkFun_RightRear_ENU_array(idx_scan,:);
    M_transform_LiDARVelodyne_to_ENU = fcn_Transform_CalculateTransformation_LiDARVelodyneToENU(GPSFront_ENU, GPSLeft_ENU, GPSRight_ENU);
    LiDAR_PointCloud_currentScan = LiDAR_PointCloud_Cell{idx_scan,1};
    LiDAR_PointCloud_currentScan_XYZ = LiDAR_PointCloud_currentScan(:,1:3);
    LiDAR_PointCloud_currentScan_ENU = se3(M_transform_LiDARVelodyne_to_ENU).transform(LiDAR_PointCloud_currentScan_XYZ);
    LiDAR_PointCloud_currentScan_Transformed = [LiDAR_PointCloud_currentScan_ENU, LiDAR_PointCloud_currentScan(:,4:end)];
    LiDAR_PointCloud_Transformed_Cell{idx_scan,1} = LiDAR_PointCloud_currentScan_Transformed;
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
% Add plot later
% if flag_do_plots
%     figure(fig_num)
%     l_x_unit = M_transform_LiDARVelodyne_to_ENU(1:3,1).';
%     l_y_unit = M_transform_LiDARVelodyne_to_ENU(1:3,2).';
%     l_z_unit = M_transform_LiDARVelodyne_to_ENU(1:3,3).';
%     LiDAROrigin = [0 0 0];
%     % Plot the Left GPS center
%     quiver3(LiDAROrigin(1,1), LiDAROrigin(1,2), LiDAROrigin(1,3), l_x_unit(1,1), l_x_unit(1,2), l_x_unit(1,3), 'Color', 'b', 'LineWidth', 1);
%     hold on
%     quiver3(LiDAROrigin(1,1), LiDAROrigin(1,2), LiDAROrigin(1,3), l_y_unit(1,1), l_y_unit(1,2), l_y_unit(1,3), 'Color', 'r', 'LineWidth', 1);
%     quiver3(LiDAROrigin(1,1), LiDAROrigin(1,2), LiDAROrigin(1,3), l_z_unit(1,1), l_z_unit(1,2), l_z_unit(1,3), 'Color', 'g', 'LineWidth', 1);
%     xlabel('X East [m]')
%     ylabel('Y North [m]')
%     zlabel('Z Up [m]')
%     legend('X-axis','Y-axis','Z-axis')
%     axis equal
% 
%     angle_XY = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(l_x_unit,l_y_unit));
%     angle_XZ = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(l_x_unit,l_z_unit));
%     angle_YZ = rad2deg(fcn_Transform_CalculateAngleBetweenVectors(l_y_unit,l_z_unit));
%     fprintf("Angle between X and Y axis is %d degree\n\n", angle_XY)
%     fprintf("Angle between X and Z axis is %d degree\n\n", angle_XZ)
%     fprintf("Angle between Y and Z axis is %d degree\n\n", angle_YZ)
% end

if flag_do_debug
    if fid ~= 0
        fprintf(fid,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
    end
end

end