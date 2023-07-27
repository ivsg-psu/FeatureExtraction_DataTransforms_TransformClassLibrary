


%% Case 1

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [2, 0, 0, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);
transform_Matrix

% transform_Matrix =
% 
%      1     0     0     2
%      0     1     0     0
%      0     0     1     0
%      0     0     0     1

%% Case 2

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5, 0, 0, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);
transform_Matrix

% transform_Matrix =
% 
%      1     0     0     5
%      0     1     0     0
%      0     0     1     0
%      0     0     0     1

%% Case 3 - wrong

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [2, 0, 0, 0, 0, 0; 5, 0, 0, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);
transform_Matrix

% transform_Matrix =
% 
%      1     0     0     2
%      0     1     0     5
%      0     0     1     0
%      0     0     0     1

