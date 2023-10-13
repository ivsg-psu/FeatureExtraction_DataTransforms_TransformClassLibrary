function VehiclePose = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, GPSFront_ENU, SensorMount_offset_relative_to_VehicleOrigin, varargin)
% fcn_Transform_findVehiclePoseinENU
%
% This function takes two GPS Antenna centers, GPSLeft_ENU and 
% GPSRight_ENU, in ENU coordinates as a (1 x 3) vector representing 
% [x, y, z] in meters, PITCH_vehicle_ENU in degrees and the sensor 
% mount's offset relative to the vehicle's origin as
% 
% [SensorMount_offset_x_relative_to_VehicleOrigin, 
%  SensorMount_offset_y_relative_to_VehicleOrigin, 
%  SensorMount_offset_z_relative_to_VehicleOrigin] 
% 
% as the inputs and outputs the vehicle pose in ENU coordinates as 
% [X_vehicle_ENU, Y_vehicle_ENU, Z_vehicle_ENU, ROLL_vehicle_ENU, 
%  PITCH_vehicle_ENU, YAW_vehicle_ENU]
%
% ASSUMPTIONS: 
%
% 1 - The ISO convention is used to output the orientation of the vehicle
%
% 2 - SensorMount_offset_y_relative_to_VehicleOrigin = 0. 
%
% 3 - The distance between the GPS Antennas mid-point and the Sensor Mount 
%     is assumed to be zero 
%
% 4 - The distance between GPSRight_ENU to GPSLeft_ENU is 2 meters. 
%
% METHOD:
%
% Step 1: Find the YAW of the vehicle relative to ENU coordinates
%        (YAW_vehicle_ENU)
%
% 1) - Find the unit vector from GPSLeft_ENU --> GPSRight_ENU
%
% 2) - Find the orthogonal by rotating the unit vector by -90 degrees via 
%      a 3x3 matrix multiplication to determine vehicle orientation in Z 
%      axis of ENU coordinates 
%
% 3) - The YAW is calculated by finding the arc tangent of the orthogonal 
%      of the unit vector. The 4-quadrant output is returned as an angle 
%      in radians using atan2.
%
% Step 2: Find the ROLL of the vehicle relative to ENU coordinates
%        (ROLL_vehicle_ENU)
%
% 1) - The vehicle is rotated based on the YAW_vehicle_ENU found in the
%      previous step. The vehicle is rotated in a way that the YAW is zero.
%
% 2) - The vehicle is rotated based on the PITCH_vehicle_ENU given in the
%      input. The vehicle is rotated in a way that the PITCH is zero.
%
% 3) - Find the dot product of horizontal unit vector when YAW, PITCH, and 
%      ROLL are zero and the unit vector when YAW and PITCH are zero from 
%      left to right, and the corresponding norms are calculated to find 
%      the magnitude of the ROLL of the vehicle. acos function is used to
%      find the angle between the two unit vectors.
%
% 4) - The direction of the ROLL is obtained by calculating the cross
%      product of the two unit vectors mentioned above (horizontal unit 
%      vector when YAW, PITCH, and ROLL are zero and the unit vector when 
%      YAW and PITCH are zero from left to right). The sign of the
%      X-coordinate of the cross product determines the direction of the
%      ROLL.
%      
% Step 3: Find the Vehicle Origin, in ENU coordinates (vehicleOrigin_ENU)
%         based on YAW.
%
% 1) - Find the mid-point of the GPSLeft_ENU and GPSRight_ENU to find the
%      position (origin) of the sensor mount. 
%
% 2) - Translate the mid-point of the GPSLeft_ENU and GPSRight_ENU to 
%      vehicle's origin by assuming the "YAW" of the sensor mount relative 
%      to the vehicle as "zero", and assuming that the center of the 
%      vehicle is "ahead" in the vehicle coordinates of the sensor mount 
%      by a known distance. The projection from sensor mount to vehicle's 
%      origin is [+X_SensorMount_center, 0, -Z_SensorMount_center] 
%      assuming that the sensor mount is [-X_SensorMount_center, 0, 
%      +Z_SensorMount_center] relative to the vehicle's origin.
%
% Step 4: Find the Vehicle Pose in ENU coordinates
%   
%  * vehiclePose_ENU = [VehicleOrigin_ENU, ROLL_vehicle_ENU, PITCH_vehicle_ENU, 
%                       YAW_vehicle_ENU]. 
%
% FORMAT:
%
%      vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(GPSLeft_ENU, GPSRight_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin)
%
% INPUTS:
%      
%      GPSLeft_ENU: the center of the left GPS Antenna [xLeft, yLeft, 
%      zLeft] in meters
%
%      GPSRight_ENU: the center of the right GPS Antenna [xRight, yRight, 
%      zRight] in meters
%
%      PITCH_vehicle_ENU: the PITCH of the vehicle in ENU coordinates in
%      degrees
%
%      SensorMount_offset_relative_to_VehicleOrigin: the position of the
%      sensor mount relative to the vehicle origin [-X_SensorMount_center,
%      0, +Z_SensorMount_center]
%                      
%
% (OPTIONAL INPUTS)   
%           
%     fig_num: The figure is plotted if fig_num is entered as the input. 
% 
% OUTPUTS:
%      
%      vehiclePose_ENU: the position of the vehicle's origin and
%      orientation of the vehicle in ENU coordinates as [X_vehicle_ENU, 
%      Y_vehicle_ENU, Z_vehicle_ENU, ROLL_vehicle_ENU, PITCH_vehicle_ENU, 
%      YAW_vehicle_ENU]
% 
% 
% DEPENDENCIES:
% 
%     None
% 
% EXAMPLES:
% 
%     See the script: script_test_fcn_findVehiclePoseinENU
%     for a full test suite.

% Revision history:
%     
% 2023_07_09: Aneesh Batchu
% -- wrote the code originally
% 2023_08_07: Aneesh Batchu
% Vectorized the code
% 2023_10_12: Xinyu Cao
% Fixed several errors in roll, pitch and yaw angle calculation, the angle
% calcialtion is incorrect from the begining
% Fixed the transformation matrix and vector transformation
% Deleted useless code, cleaned the function to speed up
% The original code was saved to fcn_Transform_findVehiclePoseinENU_OLD

% To do list:
% Edit the comments
% Fixed fcn_Transform_determineTransformMatrix function and other related
% functions
% Add comments to some new created functions

flag_do_debug = 0;  % Flag to show the results for debugging
flag_do_plots = 0;  % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking
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
    narginchk(4,5);

end

% Does user want to show the plots?
if 5 == nargin
    temp = varargin{end};
    if ~isempty(temp)
        fig_num = temp;
        figure(fig_num);
        flag_do_plots = 1; 
    end
else
    if flag_do_debug
        flag_do_plots = 1;
    end
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

[roll,pitch,yaw] = fcn_Transform_CalculateAnglesofRotation(GPSLeft_ENU,GPSRight_ENU,GPSFront_ENU);

%% Step 2 - Find the Position of the Sensor Mount and find the transormation matrix from vehicle origin to sensor mount

sensorMount_center = (GPSLeft_ENU+GPSRight_ENU)/2;
sensorMount_PoseENU = [sensorMount_center,roll,pitch, yaw];
VehicleOrigin_offset_relative_to_SensorMount = -SensorMount_offset_relative_to_VehicleOrigin;
N_points = size(sensorMount_PoseENU,1);
Mtransform_VehicleOrigin_to_SensorMount = makehgtform('translate',VehicleOrigin_offset_relative_to_SensorMount);

%% Step 3 - Calculate the Vehicle Position
for n = 1:N_points
    Mtransform_SensorMount_to_ENU = fcn_Transform_createTransformMatrix(sensorMount_PoseENU(n,:));
    Mtransform_VehicleOrigin_to_ENU = Mtransform_SensorMount_to_ENU*Mtransform_VehicleOrigin_to_SensorMount;
    VehiclePose_ENU_Homo = Mtransform_VehicleOrigin_to_ENU*[0;0;0;1];
    % VehiclePose_ENU_array(n,:) = VehiclePose_ENU_Homo(1:3).';
    VehiclePose(n,:) = [VehiclePose_ENU_Homo(1:3).',sensorMount_PoseENU(n,4:6)];

end

fprintf(1,'\nThe POSE of the vehicle in ENU coordinates is :\n');
% disp(VehiclePose);

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
    
    oriGin = [0, 0, 0];
    
    % Plot the Left GPS center
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), GPSLeft_ENU(1,1), GPSLeft_ENU(1,2), GPSLeft_ENU(1,3), 'Color', 'b', 'LineWidth', 1);
    plot3(GPSLeft_ENU(1,1), GPSLeft_ENU(1,2), GPSLeft_ENU(1,3), 'r.', 'MarkerSize', 50);
    
    % Plot the Right GPS center 
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), GPSRight_ENU(1,1), GPSRight_ENU(1,2), GPSRight_ENU(1,3), 'Color', 'g', 'LineWidth', 1);
    plot3(GPSRight_ENU(1,1), GPSRight_ENU(1,2), GPSRight_ENU(1,3), 'r.', 'MarkerSize', 50);

    % Connecting Left GPS center and Left GPS center with a line
    line_data = [GPSLeft_ENU; GPSRight_ENU];
    plot3(line_data(:,1), line_data(:,2), line_data(:,3), 'r-', 'LineWidth', 3);
    
    % Plot the unitvector from GPSLeft_to_GPSRight when YAW is zero
    quiver3(oriGin(1), oriGin(2), oriGin(3), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(1), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), yaw_to_zero_unitVector_from_GPSLeft_to_GPSRight(3), 'Color', 'green', 'LineWidth', 2);

    % Plot the unitvector from GPSLeft_to_GPSRight when YAW and PITCH are zero
    quiver3(oriGin(1), oriGin(2), oriGin(3), pitch_to_zero_unitVector_from_GPSLeft_to_GPSRight(1), pitch_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), pitch_to_zero_unitVector_from_GPSLeft_to_GPSRight(3), 'Color', 'magenta', 'LineWidth', 2);

    % Direction of the unit horizontal vector when YAW, PITCH and ROLL are zero
    quiver3(oriGin(1), oriGin(2), oriGin(3), horizontal_unitVector_left_to_right_yaw_pitch_zero(1), horizontal_unitVector_left_to_right_yaw_pitch_zero(2), horizontal_unitVector_left_to_right_yaw_pitch_zero(3), 'Color', 'red', 'LineWidth', 2);

    % Plot the unitvector from GPSLeft_to_GPSRight when YAW is zero
    quiver3(oriGin(1), oriGin(2), oriGin(3), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(1), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(3), 'Color', 'c', 'LineWidth', 2);
    
    % Find the GPSRight_ENU such that YAW of the vehicle is zero
    GPSRight_ENU_orient_to_zero = [GPSLeft_ENU(1), GPSLeft_ENU(2) + 2*roll_to_zero_unitVector_from_GPSLeft_to_GPSRight(2), GPSLeft_ENU(3)];
    plot3(GPSRight_ENU_orient_to_zero(1), GPSRight_ENU_orient_to_zero(2), GPSRight_ENU_orient_to_zero(3), 'c.', 'MarkerSize', 50);
        
    
    % Plot the direction of the unit vector 
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), unitVector_from_GPSLeft_to_GPSRight(1,1), unitVector_from_GPSLeft_to_GPSRight(2,1), unitVector_from_GPSLeft_to_GPSRight(3,1), 'Color', 'k', 'LineWidth', 1);
    
    % Plot the direction of the orthoginal of the unit vector
    quiver3(oriGin(1,1), oriGin(1,2), oriGin(1,3), rotated_unitVector_from_1to2(1,1), rotated_unitVector_from_1to2(2,1), rotated_unitVector_from_1to2(3,1), 'Color', 'r', 'LineWidth', 1);
    
    % Plot the Sensor Mount origin
    plot3(SensorMount_center(1,1), SensorMount_center(1,2), SensorMount_center(1,3), 'b.', 'MarkerSize', 50);
    
    % Plot the Vehicle origin 
    plot3(vehicleOrigin_ENU(1,1), vehicleOrigin_ENU(1,2), vehicleOrigin_ENU(1,3), 'g.', 'MarkerSize', 50);
end

if flag_do_debug
    fprintf(fileID,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end