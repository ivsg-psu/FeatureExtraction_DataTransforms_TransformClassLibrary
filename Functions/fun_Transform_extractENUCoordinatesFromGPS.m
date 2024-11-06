function [GPS_SparkFun_Front_ENU, GPS_SparkFun_LeftRear_ENU, GPS_SparkFun_RightRear_ENU,vehicleDGPS_mode] = fun_Transform_extractENUCoordinatesFromGPS(dataStructure, varargin)

% fun_Transform_extractENUCoordinatesFromGPS
% This function takes ENU calculated data strucutre as input and outputs
% ENU arrays for three GPS units and vehicleDGPS_mode
%
%
% FORMAT:
%
%      [GPS_SparkFun_Front_ENU, GPS_SparkFun_LeftRear_ENU, GPS_SparkFun_RightRear_ENU,vehicleDGPS_mode] = fun_Transform_extractENUCoordinatesFromGPS(dataStructure, (fid), (fig_num))
%
% INPUTS:
%      
%      dataStructure: cleaned and ENU filled data structure
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
%      GPS_SparkFun_Front_ENU: An array contains front GPS position in ENU
%      coordinate system
%
%      GPS_SparkFun_LeftRear_ENU: An array contains left rear GPS position in ENU
%      coordinate system
%
%      GPS_SparkFun_RightRear_ENU: An array contains right rear GPS position in ENU
%      coordinate system
%
%      vehicleDGPS_mode: An array contains DGPS mode of the vehicle
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


[cell_array_xEast,GPS_Units_names] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, 'xEast','GPS');

[cell_array_yNorth,~] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, 'yNorth','GPS');

[cell_array_zUp,~] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, 'zUp','GPS');

N_GPS_units = length(GPS_Units_names);

GPS_SparkFun_Front_ENU = [];
GPS_SparkFun_LeftRear_ENU = [];
GPS_SparkFun_RightRear_ENU = [];
% Fill the ENU arrays for all three GPS units
for idx_GPS_unit = 1:N_GPS_units
    GPS_Units_name = GPS_Units_names{idx_GPS_unit};
    array_xEast = cell_array_xEast{idx_GPS_unit};
    array_yNorth = cell_array_yNorth{idx_GPS_unit};
    array_zUp = cell_array_zUp{idx_GPS_unit};
    if strcmp(GPS_Units_name,"GPS_SparkFun_Front")
        GPS_SparkFun_Front_ENU = [array_xEast, array_yNorth, array_zUp];

    elseif strcmp(GPS_Units_name,"GPS_SparkFun_LeftRear")
        GPS_SparkFun_LeftRear_ENU = [array_xEast, array_yNorth, array_zUp];
    elseif strcmp(GPS_Units_name,"GPS_SparkFun_RightRear")
        GPS_SparkFun_RightRear_ENU = [array_xEast, array_yNorth, array_zUp];

    end
end

[cell_array_DGPS,~] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, 'DGPS_mode','GPS');
array_DGPS = cell2mat(cell_array_DGPS);
% Choose the smallest DGPS mode among three GPS units as the vehicle DGPS
% mode, smaller DGPS mode indicates poor DGPS connection
vehicleDGPS_mode = min(array_DGPS,[],2);

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



