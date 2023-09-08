
% run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')


%% Bug's Fixed - Load PoseData.mat

run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')

% The points in ENU coordinates.
sensorReading_ENU = GPS_SparkFun_LeftRear_ENU_interp;

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);


