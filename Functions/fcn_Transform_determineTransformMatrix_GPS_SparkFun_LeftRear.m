function transform_Matrix = fcn_Transform_determineTransformMatrix_GPS_SparkFun_LeftRear(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, varargin)
% fcn_Transform_determineTransformMatrix
%
% This function takes vehicle parameters, sensor pose parameters,
% sensor_or_vehicle string, vehiclePose_ENU, 
% perturbation_in_sensorPose_relative_to_SensorPlatform as the inputs and
% outputs a transform matrix, which is later used to transform the sensor
% coordinates to ENU coordinates, and ENU coordinates to sensor coordinates
%
% NOTE: The vehicle and sensor pose parameters can be loaded from
% Example_vehicleParameters_Struct
%
% METHOD (with the plots included): 
%
% Step 1: Assign the parameters to the vehicle and the sensors
%
% Step 2: The vehicle and sensors are created in the shapes of cubes.
%
% Step 3: If there are any perturbations in the position or orientation of
%         the sensors, they are added to the sensors before moving them to
%         the correct locations
%
% Step 4: Move the sensors to the corresponding locations by incorporating
%         the perturbations
%
% Step 5: Set the pose of the vehicle based on the vehiclePose_ENU
%        (input)
%
% Step 6: Find the transform matrices of the sensors and vehicle based on
%         sensor_or_vehicle string by multiplying the translation and 
%         rotation transform matrices of sensor and vehicle
%         
% ASSUMPTIONS:
%
% 1) The dimension data and the location data of the vehicle and the  
%    sensors are assumed in the current function. Need to change the data
%    according to the correct information later. 
%
% 2) All the sensors are offsetted to their correct locations relative to
%    the sensor platform
%
% 3) The sensor platform is assumed to be located at the correct location
%    without any perturbation
%
% FORMAT:
%
%      transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, 
%                          sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, varargin)
%
% INPUTS:
%
%      vehicleParameters: The structure of all vehicle parameters
%      
%           This structure includes dimensions of the vehicle and sensors, 
%           and the offsets of all the points to locate the origin. 
%
%
%      sensorPoseParameters: The structure of all sensor pose parameters
%      
%           This structure includes the position and orientation of the 
%           sensors relative to the vehicle.
%      
%
%      sensor_or_vehicle: transform matrix of this "sensor_or_vehicle" is
%                         generated to transform the "sensor_or_vehicle" 
%                         coordinates to ENU and vice versa      
%
%
%      vehiclePose_ENU: [x,y,z,roll,pitch,yaw]
% 
%        position of the vehicle:
% 
%           x: translates the vehicle in the x direction relative to
%           ENU coordinates
%           y: translates the vehicle in the y direction relative to
%           ENU coordinates
%           z: translates the vehicle in the z direction relative to
%           ENU coordinates
% 
%        orientation of the vehicle - Follows ISO convention
% 
%          roll: rotates the vehicle about its x-axis relative to ENU
%          coordinates (the vehicle's orientation changes relative to
%          the Earth's surface)
% 
%          pitch: rotates the vehicle about its y-axis relative to ENU
%          coordinates (the vehicle's orientation changes relative to
%          the Earth's surface)
% 
%          yaw: rotates the vehicle about its z-axis relative to ENU
%          coordinates (the vehicle's orientation changes relative to
%          the Earth's surface)
%
%
%      perturbation_sensor: [x_Perturbation,y_Perturbation,z_Perturbation,
%                   roll_Perturbation,pitch_Perturbation,yaw_Perturbation]
%
%        Perturbation in the position of the vehicle:
% 
%           x_Perturbation: This is the perturbation of the sensor in the x
%           direction, in cm, relative to the sensor platform coordinates.
%
%           y_Perturbation: This is the perturbation of the sensor in the y
%           direction, in cm, relative to the sensor platform coordinates.
%
%           z_Perturbation: This is the perturbation of the sensor in the z
%           direction, in cm, relative to the sensor platform coordinates.
% 
%        Perturbation IN THE orientation of the vehicle - Follows ISO 
%        convention
% 
%          roll_Perturbation: The perturbation of the sensor orientation 
%          about its x-axis 
%
%          pitch_Perturbation: The perturbation of the sensor orientation 
%          about its y-axis 
% 
%          yaw_Perturbation: The perturbation of the sensor orientation 
%          about its z-axis 
%       
%         
% 
% (OPTIONAL INPUTS)
%
%      fig_num: The figure is plotted if fig_num is entered as the input. 
%
% 
% OUTPUTS:
%      
%      transform_Matrix: transform matrix to transform the sensor
%      coordinates to ENU coordinates and vice versa
% 
% 
% DEPENDENCIES:
% 
%      None
% 
% EXAMPLES:
% 
%     See the script: script_test_fcn_Transform_determineTransformMatrix
%     for a full test suite.
%
% 2023_06_07: sbrennan@psu.edu
% -- wrote the code for hgtransform
% 2023_06_21: Aneesh Batchu
% -- started this code (functionalized)
% -- added left GPS, right GPS and Velodyne Sensor
% 2023_06_23: Aneesh Batchu
% -- added 'allsensors' case. added roll, pitch and yaw as the input
% arguments for this function. 
% 2023_06_29: Aneesh Batchu
% -- added vehiclePose_ENU as the input to change the pose of the vehicle.
% 2023_07_21: Aneesh Batchu
% -- added perturbation to the sensor position and orientation as the
% optional inputs
% 2023_07_25:Aneesh Batchu
% -- organized the code and added vehicle and sensor pose parameters as the
% inputs
% 2023_09_08: Aneesh Batchu
% -- Bug Fix: All the translations are done before rotating the sensors and
% vehicle. 

flag_do_debug = 0;  % Flag to show the results for debugging
flag_do_plots = 0;  % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking
fileID = 1; % The default file ID destination for fprintf messages

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(fileID,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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
    narginchk(5,6);

end


if 6 == nargin
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


%% If there is any perturbation, add it to the sensor position and orientation before moving the items to their correct locations 

% This function outputs the structure of the sensorPose perturbation of
% each sensor as arrays (6*1 matrices)
sensorPose_Perturbation = fcn_Transform_setPerturbationToSensorPose(sensor_or_vehicle,perturbation_in_sensorPose_relative_to_SensorPlatform);

%% Now, find the transform matrices to move items to their correct locations

%-------------------- GPS_Hemisphere_SensorPlatform ----------------------|
% Transform matrices to move the GPS_Hemisphere_SensorPlatform to its correct
% location
Mtransform_GPS_Hemisphere_SensorPlatform_Rear_translate = makehgtform('translate',[sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_x_relative_to_vehicle, ...
                                                                                   sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_y_relative_to_vehicle, ...
                                                                                   sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_z_relative_to_vehicle]);

Mtransform_GPS_Hemisphere_SensorPlatform_Rear_zrotate = makehgtform('zrotate',sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.yaw_relative_to_own_axis * pi/180);
Mtransform_GPS_Hemisphere_SensorPlatform_Rear_yrotate = makehgtform('yrotate',sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.pitch_relative_to_own_axis * pi/180);
Mtransform_GPS_Hemisphere_SensorPlatform_Rear_xrotate = makehgtform('xrotate',sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.roll_relative_to_own_axis * pi/180);


%--------------------------- Lidar_Sick_Rear ----------------------------|
% Offset values to translate the Lidar_Sick_Rear to its correct location
Lidar_Sick_Rear_offset_x_relative_to_sensorplatform = sensorPoseParameters.Lidar_Sick_Rear.offset_x_relative_to_sensorplatform + sensorPose_Perturbation.Lidar_Sick_Rear(1)*(1/100); % Meters - GUESS!!
Lidar_Sick_Rear_offset_y_relative_to_sensorplatform = sensorPoseParameters.Lidar_Sick_Rear.offset_y_relative_to_sensorplatform + sensorPose_Perturbation.Lidar_Sick_Rear(2)*(1/100); % Meters - GUESS!!
Lidar_Sick_Rear_offset_z_relative_to_sensorplatform = sensorPoseParameters.Lidar_Sick_Rear.offset_z_relative_to_sensorplatform + sensorPose_Perturbation.Lidar_Sick_Rear(3)*(1/100); % Meters - GUESS!!

% Transform matrices to move the Lidar_Sick_Rear to its correct location
Mtransform_Lidar_Sick_Rear_translate = makehgtform('translate',[Lidar_Sick_Rear_offset_x_relative_to_sensorplatform, ...
                                                                Lidar_Sick_Rear_offset_y_relative_to_sensorplatform, ...
                                                                Lidar_Sick_Rear_offset_z_relative_to_sensorplatform]);

Mtransform_Lidar_Sick_Rear_zrotate = makehgtform('zrotate',(sensorPoseParameters.Lidar_Sick_Rear.yaw_relative_to_own_axis + sensorPose_Perturbation.Lidar_Sick_Rear(6)) * pi/180);
Mtransform_Lidar_Sick_Rear_yrotate = makehgtform('yrotate',(sensorPoseParameters.Lidar_Sick_Rear.pitch_relative_to_own_axis + sensorPose_Perturbation.Lidar_Sick_Rear(5)) * pi/180);
Mtransform_Lidar_Sick_Rear_xrotate = makehgtform('xrotate',(sensorPoseParameters.Lidar_Sick_Rear.roll_relative_to_own_axis + sensorPose_Perturbation.Lidar_Sick_Rear(4)) * pi/180);


%------------------------- GPS_SparkFun_LeftRear -------------------------|
% Offset values to translate the GPS_SparkFun_LeftRear to its correct location
GPS_SparkFun_LeftRear_offset_x_relative_to_sensorplatform =  sensorPoseParameters.GPS_SparkFun_LeftRear.offset_x_relative_to_sensorplatform + sensorPose_Perturbation.GPS_SparkFun_LeftRear(1)*(1/100); % Meters - GUESS!!
GPS_SparkFun_LeftRear_offset_y_relative_to_sensorplatform =  sensorPoseParameters.GPS_SparkFun_LeftRear.offset_y_relative_to_sensorplatform + sensorPose_Perturbation.GPS_SparkFun_LeftRear(2)*(1/100); % Meters - GUESS!!
GPS_SparkFun_LeftRear_offset_z_relative_to_sensorplatform =  sensorPoseParameters.GPS_SparkFun_LeftRear.offset_z_relative_to_sensorplatform + sensorPose_Perturbation.GPS_SparkFun_LeftRear(3)*(1/100); % Meters - GUESS!!

% Transform matrices to move the GPS_SparkFun_LeftRear to its correct location
Mtransform_GPS_SparkFun_LeftRear_translate = makehgtform('translate',[GPS_SparkFun_LeftRear_offset_x_relative_to_sensorplatform, ...
                                                                      GPS_SparkFun_LeftRear_offset_y_relative_to_sensorplatform, ...
                                                                      GPS_SparkFun_LeftRear_offset_z_relative_to_sensorplatform]);

Mtransform_GPS_SparkFun_LeftRear_zrotate = makehgtform('zrotate',(sensorPoseParameters.GPS_SparkFun_LeftRear.yaw_relative_to_own_axis + sensorPose_Perturbation.GPS_SparkFun_LeftRear(6)) * pi/180);
Mtransform_GPS_SparkFun_LeftRear_yrotate = makehgtform('yrotate',(sensorPoseParameters.GPS_SparkFun_LeftRear.pitch_relative_to_own_axis + sensorPose_Perturbation.GPS_SparkFun_LeftRear(5)) * pi/180);
Mtransform_GPS_SparkFun_LeftRear_xrotate = makehgtform('xrotate',(sensorPoseParameters.GPS_SparkFun_LeftRear.roll_relative_to_own_axis + sensorPose_Perturbation.GPS_SparkFun_LeftRear(4)) * pi/180);


%------------------------- GPS_SparkFun_RightRear -------------------------|
% Offset values to translate the GPS_SparkFun_RightRear to its correct location
GPS_SparkFun_RightRear_offset_x_relative_to_sensorplatform =  sensorPoseParameters.GPS_SparkFun_RightRear.offset_x_relative_to_sensorplatform + sensorPose_Perturbation.GPS_SparkFun_RightRear(1)*(1/100); % Meters - GUESS!!
GPS_SparkFun_RightRear_offset_y_relative_to_sensorplatform =  sensorPoseParameters.GPS_SparkFun_RightRear.offset_y_relative_to_sensorplatform + sensorPose_Perturbation.GPS_SparkFun_RightRear(2)*(1/100); % Meters - GUESS!!
GPS_SparkFun_RightRear_offset_z_relative_to_sensorplatform =  sensorPoseParameters.GPS_SparkFun_RightRear.offset_z_relative_to_sensorplatform + sensorPose_Perturbation.GPS_SparkFun_RightRear(3)*(1/100); % Meters - GUESS!!

% Transform matrices to move the GPS_SparkFun_RightRear to its correct location
Mtransform_GPS_SparkFun_RightRear_translate = makehgtform('translate',[GPS_SparkFun_RightRear_offset_x_relative_to_sensorplatform, ...
                                                         GPS_SparkFun_RightRear_offset_y_relative_to_sensorplatform, ...
                                                         GPS_SparkFun_RightRear_offset_z_relative_to_sensorplatform]);

Mtransform_GPS_SparkFun_RightRear_zrotate = makehgtform('zrotate',(sensorPoseParameters.GPS_SparkFun_RightRear.yaw_relative_to_own_axis + sensorPose_Perturbation.GPS_SparkFun_RightRear(6)) * pi/180);
Mtransform_GPS_SparkFun_RightRear_yrotate = makehgtform('yrotate',(sensorPoseParameters.GPS_SparkFun_RightRear.pitch_relative_to_own_axis + sensorPose_Perturbation.GPS_SparkFun_RightRear(5)) * pi/180);
Mtransform_GPS_SparkFun_RightRear_xrotate = makehgtform('xrotate',(sensorPoseParameters.GPS_SparkFun_RightRear.roll_relative_to_own_axis + sensorPose_Perturbation.GPS_SparkFun_RightRear(4)) * pi/180);


%-------------------------- Lidar_Velodyne_Rear --------------------------|
% Offset values to translate the Lidar_Velodyne_Rear to its correct location
Lidar_Velodyne_Rear_offset_x_relative_to_sensorplatform = sensorPoseParameters.Lidar_Velodyne_Rear.offset_x_relative_to_sensorplatform + sensorPose_Perturbation.Lidar_Velodyne_Rear(1)*(1/100); % Meters - GUESS!!
Lidar_Velodyne_Rear_offset_y_relative_to_sensorplatform = sensorPoseParameters.Lidar_Velodyne_Rear.offset_y_relative_to_sensorplatform + sensorPose_Perturbation.Lidar_Velodyne_Rear(2)*(1/100); % Meters - GUESS!!
Lidar_Velodyne_Rear_offset_z_relative_to_sensorplatform = sensorPoseParameters.Lidar_Velodyne_Rear.offset_z_relative_to_sensorplatform + sensorPose_Perturbation.Lidar_Velodyne_Rear(3)*(1/100); % Meters - GUESS!!

% Transform matrices to move the Lidar_Velodyne_Rear to its correct location
Mtransform_Lidar_Velodyne_Rear_translate = makehgtform('translate',[Lidar_Velodyne_Rear_offset_x_relative_to_sensorplatform, ...
                                                                    Lidar_Velodyne_Rear_offset_y_relative_to_sensorplatform, ...
                                                                    Lidar_Velodyne_Rear_offset_z_relative_to_sensorplatform]);

Mtransform_Lidar_Velodyne_Rear_zrotate = makehgtform('zrotate',(sensorPoseParameters.Lidar_Velodyne_Rear.yaw_relative_to_own_axis + sensorPose_Perturbation.Lidar_Velodyne_Rear(6)) * pi/180);
Mtransform_Lidar_Velodyne_Rear_yrotate = makehgtform('yrotate',(sensorPoseParameters.Lidar_Velodyne_Rear.pitch_relative_to_own_axis + sensorPose_Perturbation.Lidar_Velodyne_Rear(5)) * pi/180);
Mtransform_Lidar_Velodyne_Rear_xrotate = makehgtform('xrotate',(sensorPoseParameters.Lidar_Velodyne_Rear.roll_relative_to_own_axis + sensorPose_Perturbation.Lidar_Velodyne_Rear(4)) * pi/180);

%% Set the pose of the vehicle in ENU coordinates and Find the transform matrix


% First, translate the vehicle along x, y, and z
% Next, rotate the vehicle based on yaw, pitch and roll 
% The vehicle is rotated based on the ISO convention

Nrows = size(vehiclePose_ENU,1);
transform_Matrix = zeros(4,4,Nrows);

for i = 1:Nrows

    Vehicle_x_relative_to_ENUCoord = vehiclePose_ENU(i,1);
    Vehicle_y_relative_to_ENUCoord = vehiclePose_ENU(i,2);
    Vehicle_z_relative_to_ENUCoord = vehiclePose_ENU(i,3);
    Vehicle_roll_relative_to_own_axis = -vehiclePose_ENU(i,4);
    Vehicle_pitch_relative_to_own_axis = vehiclePose_ENU(i,5);
    Vehicle_yaw_relative_to_own_axis = vehiclePose_ENU(i,6);


    Mtransform_Vehicle_translate = makehgtform('translate', [Vehicle_x_relative_to_ENUCoord Vehicle_y_relative_to_ENUCoord Vehicle_z_relative_to_ENUCoord]);
    Mtransform_Vehicle_zrotate = makehgtform('zrotate',Vehicle_yaw_relative_to_own_axis*pi/180);
    Mtransform_Vehicle_yrotate = makehgtform('yrotate',Vehicle_pitch_relative_to_own_axis*pi/180);
    Mtransform_Vehicle_xrotate = makehgtform('xrotate',Vehicle_roll_relative_to_own_axis*pi/180);

    % transform_Matrix_vehicleCase = Mtransform_Vehicle_translate*Mtransform_Vehicle_zrotate*Mtransform_Vehicle_yrotate*Mtransform_Vehicle_xrotate;

    % transform_Matrix_GPS_Hemisphere_SensorPlatform_Rear = Mtransform_GPS_Hemisphere_SensorPlatform_Rear_translate*Mtransform_GPS_Hemisphere_SensorPlatform_Rear_zrotate*...
    %             Mtransform_GPS_Hemisphere_SensorPlatform_Rear_yrotate*Mtransform_GPS_Hemisphere_SensorPlatform_Rear_xrotate;
    % Find the transform matrix

    
    % transform_Matrix(:,:,i) = transform_Matrix_GPS_Hemisphere_SensorPlatform_Rear*Mtransform_GPS_SparkFun_LeftRear_translate*transform_Matrix_vehicleCase*...
    %                           Mtransform_GPS_SparkFun_LeftRear_zrotate*Mtransform_GPS_SparkFun_LeftRear_yrotate*Mtransform_GPS_SparkFun_LeftRear_xrotate;
    
    transform_Matrix(:,:,i) = Mtransform_Vehicle_translate*Mtransform_GPS_Hemisphere_SensorPlatform_Rear_translate*Mtransform_GPS_SparkFun_LeftRear_translate*...
                              Mtransform_Vehicle_zrotate*Mtransform_Vehicle_yrotate*Mtransform_Vehicle_xrotate*...
                              Mtransform_GPS_Hemisphere_SensorPlatform_Rear_zrotate*Mtransform_GPS_Hemisphere_SensorPlatform_Rear_yrotate*Mtransform_GPS_Hemisphere_SensorPlatform_Rear_xrotate*...
                              Mtransform_GPS_SparkFun_LeftRear_zrotate*Mtransform_GPS_SparkFun_LeftRear_yrotate*Mtransform_GPS_SparkFun_LeftRear_xrotate;
    
    % transform_Matrix(:,:,i) = transform_Matrix_GPS_Hemisphere_SensorPlatform_Rear*Mtransform_GPS_SparkFun_LeftRear_translate*Mtransform_Vehicle_translate*...
    %                           Mtransform_Vehicle_zrotate*Mtransform_GPS_SparkFun_LeftRear_zrotate*Mtransform_Vehicle_yrotate*Mtransform_GPS_SparkFun_LeftRear_yrotate*Mtransform_Vehicle_xrotate*Mtransform_GPS_SparkFun_LeftRear_xrotate;


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
    
    % Vehicle body
    cube_points = fcn_INTERNAL_fillCube(vehicleParameters.Vehicle.length_in_meters,vehicleParameters.Vehicle.width_in_meters,vehicleParameters.Vehicle.height_in_meters, ...
                  vehicleParameters.Vehicle.offset_length_in_meters,vehicleParameters.Vehicle.offset_width_in_meters,vehicleParameters.Vehicle.offset_height_in_meters);
    % Plot the result and make the body the parent
    mapping_van_plot_handles = fcn_INTERNAL_plotCube(cube_points);

    % Once the first plot is made, we can define the axes
    ax = gca; % Grab current axes;
    handles.transform_ground_to_body = hgtransform('Parent',ax);
    handles.transform_body_to_sensorplatform = hgtransform('Parent',handles.transform_ground_to_body);
    handles.transform_sensorplatform_to_LIDAR = hgtransform('Parent',handles.transform_body_to_sensorplatform);
    handles.transform_sensorplatform_to_leftGPS = hgtransform('Parent',handles.transform_body_to_sensorplatform);
    handles.transform_sensorplatform_to_rightGPS = hgtransform('Parent',handles.transform_body_to_sensorplatform);
    handles.transform_sensorplatform_to_velodyneLIDAR = hgtransform('Parent',handles.transform_body_to_sensorplatform);
    set(mapping_van_plot_handles,'Parent',handles.transform_ground_to_body); 


    % GPS Hemisphere SensorPlatform
    sensor_box_points = fcn_INTERNAL_fillCube(vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.length_in_meters,vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.width_in_meters, ...
                        vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.height_in_meters,vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_length_in_meters, ...
                        vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_width_in_meters,vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_height_in_meters);
    % Plot the result and make the sensorplatform the parent
    sensor_platform_plot_handles = fcn_INTERNAL_plotCube(sensor_box_points);
    set(sensor_platform_plot_handles,'Parent',handles.transform_body_to_sensorplatform);


    % Lidar Sick Rear 
    lidar_SICK_box_points = fcn_INTERNAL_fillCube(vehicleParameters.Lidar_Sick_Rear.length_in_meters,vehicleParameters.Lidar_Sick_Rear.width_in_meters, ...
                            vehicleParameters.Lidar_Sick_Rear.height_in_meters,vehicleParameters.Lidar_Sick_Rear.offset_length_in_meters, ...
                            vehicleParameters.Lidar_Sick_Rear.offset_width_in_meters,vehicleParameters.Lidar_Sick_Rear.offset_height_in_meters);
    % Plot the result and make the LIDAR the parent
    lidar_sick_plot_handles = fcn_INTERNAL_plotCube(lidar_SICK_box_points);
    set(lidar_sick_plot_handles,'Parent',handles.transform_sensorplatform_to_LIDAR);


    % GPS SparkFun LeftRear 
    GPS_left_box_points = fcn_INTERNAL_fillCube(vehicleParameters.GPS_SparkFun_LeftRear.length_in_meters,vehicleParameters.GPS_SparkFun_LeftRear.width_in_meters, ...
                          vehicleParameters.GPS_SparkFun_LeftRear.height_in_meters,vehicleParameters.GPS_SparkFun_LeftRear.offset_length_in_meters, ...
                          vehicleParameters.GPS_SparkFun_LeftRear.offset_width_in_meters,vehicleParameters.GPS_SparkFun_LeftRear.offset_height_in_meters);
    % Plot the result and make the leftGPS the parent
    GPS_left_plot_handles = fcn_INTERNAL_plotCube(GPS_left_box_points);
    % set(GPS_left_plot_handles,'Parent',handles.transform_GPSchannel_to_leftGPS);
    set(GPS_left_plot_handles,'Parent',handles.transform_sensorplatform_to_leftGPS);


    % GPS SparkFun RightRear
    GPS_right_box_points = fcn_INTERNAL_fillCube(vehicleParameters.GPS_SparkFun_RightRear.length_in_meters,vehicleParameters.GPS_SparkFun_RightRear.width_in_meters, ...
                          vehicleParameters.GPS_SparkFun_RightRear.height_in_meters,vehicleParameters.GPS_SparkFun_RightRear.offset_length_in_meters, ...
                          vehicleParameters.GPS_SparkFun_RightRear.offset_width_in_meters,vehicleParameters.GPS_SparkFun_RightRear.offset_height_in_meters);
    % Plot the result and make the RightGPS the parent
    GPS_right_plot_handles = fcn_INTERNAL_plotCube(GPS_right_box_points);
    % set(GPS_right_plot_handles,'Parent',handles.transform_GPSchannel_to_rightGPS);
    set(GPS_right_plot_handles,'Parent',handles.transform_sensorplatform_to_rightGPS);


    % Lidar Velodyne Rear
    lidar_velodyne_box_points = fcn_INTERNAL_fillCube(vehicleParameters.Lidar_Velodyne_Rear.length_in_meters,vehicleParameters.Lidar_Velodyne_Rear.width_in_meters, ...
                                vehicleParameters.Lidar_Velodyne_Rear.height_in_meters,vehicleParameters.Lidar_Velodyne_Rear.offset_length_in_meters, ...
                                vehicleParameters.Lidar_Velodyne_Rear.offset_width_in_meters,vehicleParameters.Lidar_Velodyne_Rear.offset_height_in_meters);
    % Plot the result and make the LIDAR the parent
    lidar_velodyne_plot_handles = fcn_INTERNAL_plotCube(lidar_velodyne_box_points);
    set(lidar_velodyne_plot_handles,'Parent',handles.transform_sensorplatform_to_velodyneLIDAR);

    % Now, start moving items to their correct locations

    % Move GPS_Hemisphere_SensorPlatform to its correct location
    set(handles.transform_body_to_sensorplatform,'Matrix',Mtransform_GPS_Hemisphere_SensorPlatform_Rear_translate*Mtransform_GPS_Hemisphere_SensorPlatform_Rear_zrotate* ...
                                                          Mtransform_GPS_Hemisphere_SensorPlatform_Rear_yrotate*Mtransform_GPS_Hemisphere_SensorPlatform_Rear_xrotate);


    % Move the Lidar_Sick_Rear to its correct location
    set(handles.transform_sensorplatform_to_LIDAR,'Matrix',Mtransform_Lidar_Sick_Rear_translate*Mtransform_Lidar_Sick_Rear_zrotate* ...
                                                           Mtransform_Lidar_Sick_Rear_yrotate*Mtransform_Lidar_Sick_Rear_xrotate);

    % Move the GPS_SparkFun_LeftRear to its correct location
    set(handles.transform_sensorplatform_to_leftGPS,'Matrix',Mtransform_GPS_SparkFun_LeftRear_translate*Mtransform_GPS_SparkFun_LeftRear_zrotate* ...
                                                             Mtransform_GPS_SparkFun_LeftRear_yrotate*Mtransform_GPS_SparkFun_LeftRear_xrotate);
    
    % Move the GPS_SparkFun_RightRear to its correct location
    set(handles.transform_sensorplatform_to_rightGPS,'Matrix',Mtransform_GPS_SparkFun_RightRear_translate*Mtransform_GPS_SparkFun_RightRear_zrotate*Mtransform_GPS_SparkFun_RightRear_yrotate* ...
                                                              Mtransform_GPS_SparkFun_RightRear_xrotate);

    % Move the Lidar_Velodyne_Rear to its correct location
    set(handles.transform_sensorplatform_to_velodyneLIDAR,'Matrix',Mtransform_Lidar_Velodyne_Rear_translate*Mtransform_Lidar_Velodyne_Rear_zrotate* ...
                                                                   Mtransform_Lidar_Velodyne_Rear_yrotate*Mtransform_Lidar_Velodyne_Rear_xrotate);

    % Move the Vehicle to its correct location
    set(handles.transform_ground_to_body,'Matrix',Mtransform_Vehicle_translate*Mtransform_Vehicle_zrotate*Mtransform_Vehicle_yrotate*Mtransform_Vehicle_xrotate);


end

if flag_do_debug
    fprintf(fileID,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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

function cube_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width, offset_height)
% fprintf(1,'Preparing cube of:\n');
% fprintf(1,'\tLength: %.2f\n',length);
% fprintf(1,'\tWidth: %.2f\n',width);
% fprintf(1,'\tHeigth: %.2f\n',height);
cube_points = [
    -length -width -height
    length -width -height
    nan     nan    nan
    -length  width -height
    length  width -height
    nan     nan    nan
    -length -width  height
    length -width  height
    nan     nan    nan
    -length  width  height
    length  width  height
    nan     nan    nan

    -length -width -height
    -length +width -height
    nan     nan    nan
    +length -width -height
    +length +width -height
    nan     nan    nan
    -length -width  height
    -length +width  height
    nan     nan    nan
    +length -width  height
    +length +width  height
    nan     nan    nan

    -length -width -height
    -length -width +height
    nan     nan    nan
    +length -width -height
    +length -width +height
    nan     nan    nan
    -length +width -height
    -length +width +height
    nan     nan    nan
    +length +width -height
    +length +width +height
    nan     nan    nan
    0 0 0
    ]*0.5;
cube_points(1:end-1,:) = cube_points(1:end-1,:)+[offset_length offset_width offset_height];
end

function plot_handles = fcn_INTERNAL_plotCube(cube_points)
% See help on hgtransform to understand why we are saving the handles
% Plot the edges
plot_handles(1) = plot3(cube_points(1:end-1,1),cube_points(1:end-1,2),cube_points(1:end-1,3),'.-','MarkerSize',20);
color_plot = get(plot_handles(1),'Color');
hold on;
axis equal;
grid on;
xlim([-15 15]);
ylim([-15 15]);

xticks(-15:1:15);
yticks(-15:1:15);
%zlim([-15 15]);
xlabel('xEast','FontSize',15,'FontWeight','bold');
ylabel('yNorth','FontSize',15,'FontWeight','bold');
zlabel('zUp','FontSize',15,'FontWeight','bold');

% Plot the origin
plot_handles(2) = plot3(cube_points(end,1),cube_points(end,2),cube_points(end,3),'Color',color_plot,'MarkerSize',50);

% Plot the coordinate system
most_negative_point = cube_points(1,:);
most_positive_point = cube_points(end-2,:);
differences = most_positive_point-most_negative_point;
mean_differences = mean(differences);
differences = [mean_differences mean_differences mean_differences];

% Plot x-axis
plot_handles(3) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    differences(1),0,0,'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [differences(1),0,0];
plot_handles(4) = text(text_point(1),text_point(2),text_point(3),'X','Color',color_plot);

% Plot y-axis
plot_handles(5) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,differences(2),0,'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [0,differences(2),0];
plot_handles(6) = text(text_point(1),text_point(2),text_point(3),'Y','Color',color_plot);

% Plot z-axis
plot_handles(7) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,0,differences(3),'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [0,0,differences(3)];
plot_handles(8) = text(text_point(1),text_point(2),text_point(3),'Z','Color',color_plot);


end

