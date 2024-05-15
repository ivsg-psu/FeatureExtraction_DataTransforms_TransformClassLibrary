function XYZ_Lidar_in_ENU_array = fcn_Transform_SickLidarCoordToENU(GPSLeft_ENU_array, GPSRight_ENU_array, GPSFront_ENU_array,XYZ_Lidar_array, sensorPoseParameters,calibrate_matrix)

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
% 2 - RearGPSCenter_offset_relative_to_VehicleOrigin = 0. 
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
% Fixed several errors in roll, pitch and yaw angle calculations, the angle
% calculation is incorrect from the begining
% Fixed the transformation matrix and vector transformation
% Deleted useless code, cleaned the function to speed up
% The original code was saved to fcn_Transform_findVehiclePoseinENU_OLD
% 2024_02_11: Xinyu Cao
% Add comments, and clean the function

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
    narginchk(4,4);

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


N_points = size(sensorMount_PoseENU,1);

%% Step 2 - Calculate the Vehicle Position
for idx_point = 1:N_points
    GPSLeft_ENU = GPSLeft_ENU_array(idx_point,:);
    GPSRight_ENU = GPSRight_ENU_array(idx_point,:);
    GPSFront_ENU = GPSFront_ENU_array(idx_point,:);
    XYZ_Lidar = XYZ_Lidar_array(idx_point,:);
    Mtransform_LidarSickRear_to_ENU = fcn_Transform_CalculateTransformation_RearSickLidarToENU(GPSLeft_ENU, GPSRight_ENU, GPSFront_ENU,calibrate_matrix);
    XYZ_Lidar_in_ENU_homo = Mtransform_LidarSickRear_to_ENU*[XYZ_Lidar.';1];
    XYZ_Lidar_in_ENU = XYZ_Lidar_in_ENU_homo(1:3).';
    XYZ_Lidar_in_ENU_array(idx_point,:) = XYZ_Lidar_in_ENU;
    % VehiclePose(idx_point,:) = [VehiclePose_ENU_Homo(1:3).',[roll,pitch,yaw]];
end
fprintf(1,'\nThe POSE of the vehicle in ENU coordinates is :\n');
% disp(VehiclePose);