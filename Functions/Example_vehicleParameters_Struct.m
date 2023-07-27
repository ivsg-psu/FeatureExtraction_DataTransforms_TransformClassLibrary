clc
clear vehicleParameters sensorPoseParameters

% The shapes of all the sensors and the vehicle's body are assumed to be 
% cuboid

% All the parameters are in meters

% The offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points (edges) except the origin.

% Vehicle parameters
% inches times 1ft = 12inches times 1 meter = 3.281 feet

vehicleParameters.Vehicle.length_in_meters = 4.825;
vehicleParameters.Vehicle.width_in_meters = 2.137;
vehicleParameters.Vehicle.height_in_meters = 1.827;

vehicleParameters.Vehicle.offset_length_in_meters = vehicleParameters.Vehicle.length_in_meters/2 - 0.878;
vehicleParameters.Vehicle.offset_width_in_meters = 0;
tire_offset_height = 15*(1/12)*(1/3.281);
vehicleParameters.Vehicle.offset_height_in_meters = vehicleParameters.Vehicle.height_in_meters/2 - tire_offset_height;

% GPS Hemisphere SensorPlatform Rear
% inches times 1ft = 12inches times 1 meter = 3.281 feet

vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.length_in_meters = 2*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.width_in_meters = 60*(1/12)*(1/3.281); 
vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.height_in_meters = 2.5*(1/12)*(1/3.281);

vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_length_in_meters = 0;
vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_width_in_meters = 0;
vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_height_in_meters = vehicleParameters.GPS_Hemisphere_SensorPlatform_Rear.height_in_meters/2;

% Lidar Sick Rear 
% inches times 1ft = 12inches times 1 meter = 3.281 feet

vehicleParameters.Lidar_Sick_Rear.length_in_meters = 10*(1/12)*(1/3.281);
vehicleParameters.Lidar_Sick_Rear.width_in_meters = 10*(1/12)*(1/3.281);
vehicleParameters.Lidar_Sick_Rear.height_in_meters = 10*(1/12)*(1/3.281);

vehicleParameters.Lidar_Sick_Rear.offset_length_in_meters = 0;
vehicleParameters.Lidar_Sick_Rear.offset_width_in_meters = 0;
vehicleParameters.Lidar_Sick_Rear.offset_height_in_meters = 0;

% GPS SparkFun LeftRear 
% inches times 1ft = 12inches times 1 meter = 3.281 feet

vehicleParameters.GPS_SparkFun_LeftRear.length_in_meters = 15*(1/12)*(1/3.281);
vehicleParameters.GPS_SparkFun_LeftRear.width_in_meters = 15*(1/12)*(1/3.281);
vehicleParameters.GPS_SparkFun_LeftRear.height_in_meters = 3*(1/12)*(1/3.281);

vehicleParameters.GPS_SparkFun_LeftRear.offset_length_in_meters = 0;
vehicleParameters.GPS_SparkFun_LeftRear.offset_width_in_meters = 0;
vehicleParameters.GPS_SparkFun_LeftRear.offset_height_in_meters = -vehicleParameters.GPS_SparkFun_LeftRear.height_in_meters/2;

% GPS SparkFun RightRear 
% inches times 1ft = 12inches times 1 meter = 3.281 feet

vehicleParameters.GPS_SparkFun_RightRear.length_in_meters = 15*(1/12)*(1/3.281);
vehicleParameters.GPS_SparkFun_RightRear.width_in_meters = 15*(1/12)*(1/3.281);
vehicleParameters.GPS_SparkFun_RightRear.height_in_meters = 3*(1/12)*(1/3.281);

vehicleParameters.GPS_SparkFun_RightRear.offset_length_in_meters = 0;
vehicleParameters.GPS_SparkFun_RightRear.offset_width_in_meters = 0;
vehicleParameters.GPS_SparkFun_RightRear.offset_height_in_meters = -vehicleParameters.GPS_SparkFun_RightRear.height_in_meters/2;

% Lidar Velodyne Rear 
% inches times 1ft = 12inches times 1 meter = 3.281 feet

vehicleParameters.Lidar_Velodyne_Rear.length_in_meters = 10*(1/12)*(1/3.281);
vehicleParameters.Lidar_Velodyne_Rear.width_in_meters = 10*(1/12)*(1/3.281);
vehicleParameters.Lidar_Velodyne_Rear.height_in_meters = 10*(1/12)*(1/3.281);

vehicleParameters.Lidar_Velodyne_Rear.offset_length_in_meters = 0;
vehicleParameters.Lidar_Velodyne_Rear.offset_width_in_meters = -(vehicleParameters.Lidar_Velodyne_Rear.width_in_meters/2)+1*(1/12)*(1/3.281);
vehicleParameters.Lidar_Velodyne_Rear.offset_height_in_meters = 0;


% Sensor Pose Parameters. 

% GPS_Hemisphere_SensorPlatform
sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_x_relative_to_vehicle = -1;
sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_y_relative_to_vehicle = 0;
sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.offset_z_relative_to_vehicle = 1.6;
sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.roll_relative_to_own_axis = 0;
sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.pitch_relative_to_own_axis = 0;
sensorPoseParameters.GPS_Hemisphere_SensorPlatform_Rear.yaw_relative_to_own_axis = 0;


% Lidar Sick Rear 
sensorPoseParameters.Lidar_Sick_Rear.offset_x_relative_to_sensorplatform = -0.4;
sensorPoseParameters.Lidar_Sick_Rear.offset_y_relative_to_sensorplatform =  0;
sensorPoseParameters.Lidar_Sick_Rear.offset_z_relative_to_sensorplatform = -0.1;
sensorPoseParameters.Lidar_Sick_Rear.roll_relative_to_own_axis = 0;
sensorPoseParameters.Lidar_Sick_Rear.pitch_relative_to_own_axis = 90;
sensorPoseParameters.Lidar_Sick_Rear.yaw_relative_to_own_axis = 0;


% GPS SparkFun LeftRear
sensorPoseParameters.GPS_SparkFun_LeftRear.offset_x_relative_to_sensorplatform =  0.3; 
sensorPoseParameters.GPS_SparkFun_LeftRear.offset_y_relative_to_sensorplatform =  30*(1/12)*(1/3.281);
sensorPoseParameters.GPS_SparkFun_LeftRear.offset_z_relative_to_sensorplatform =  0.5; 
sensorPoseParameters.GPS_SparkFun_LeftRear.roll_relative_to_own_axis = 0;
sensorPoseParameters.GPS_SparkFun_LeftRear.pitch_relative_to_own_axis = 0;
sensorPoseParameters.GPS_SparkFun_LeftRear.yaw_relative_to_own_axis = 0;


% GPS SparkFun RightRear
sensorPoseParameters.GPS_SparkFun_RightRear.offset_x_relative_to_sensorplatform =  0.3;
sensorPoseParameters.GPS_SparkFun_RightRear.offset_y_relative_to_sensorplatform =  -30*(1/12)*(1/3.281); % Meters - GUESS!!
sensorPoseParameters.GPS_SparkFun_RightRear.offset_z_relative_to_sensorplatform =  0.5; % Meters - GUESS!!
sensorPoseParameters.GPS_SparkFun_RightRear.roll_relative_to_own_axis = 0;
sensorPoseParameters.GPS_SparkFun_RightRear.pitch_relative_to_own_axis = 0;
sensorPoseParameters.GPS_SparkFun_RightRear.yaw_relative_to_own_axis = 0;


% Lidar Velodyne Rear 
sensorPoseParameters.Lidar_Velodyne_Rear.offset_x_relative_to_sensorplatform = -1.0; % Meters - GUESS!!
sensorPoseParameters.Lidar_Velodyne_Rear.offset_y_relative_to_sensorplatform =  0; % Meters - GUESS!!
sensorPoseParameters.Lidar_Velodyne_Rear.offset_z_relative_to_sensorplatform = 0.4; % Meters - GUESS!!
sensorPoseParameters.Lidar_Velodyne_Rear.roll_relative_to_own_axis = -36;
sensorPoseParameters.Lidar_Velodyne_Rear.pitch_relative_to_own_axis = 0;
sensorPoseParameters.Lidar_Velodyne_Rear.yaw_relative_to_own_axis = 90;


