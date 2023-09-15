
% run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')


%% Bug's Fixed - Load PoseData.mat

load PoseData_1.mat

% figure(343434)
% plot3(GPS_SparkFun_Temp_ENU_interp(:,1),GPS_SparkFun_Temp_ENU_interp(:,2),GPS_SparkFun_Temp_ENU_interp(:,3),'b-','Linewidth',50);
% hold on
% grid on
% box on
% plot3(GPS_SparkFun_LeftRear_ENU_interp(:,1),GPS_SparkFun_LeftRear_ENU_interp(:,2),GPS_SparkFun_LeftRear_ENU_interp(:,3),'r-','Linewidth',50);
% plot3(GPS_SparkFun_RightRear_ENU_interp(:,1),GPS_SparkFun_RightRear_ENU_interp(:,2),GPS_SparkFun_RightRear_ENU_interp(:,3),'k-','Linewidth',50);
% 
% xlabel('xEast', 'FontSize', 15);
% ylabel('yNorth', 'FontSize', 15);
% zlabel('zUp', 'FontSize', 15);

% Temp_GPS = GPS_SparkFun_Temp_ENU_interp - vehiclePose_ENU(:,1:3);
% Left_GPS = GPS_SparkFun_LeftRear_ENU_interp - vehiclePose_ENU(:,1:3);
% Right_GPS = GPS_SparkFun_RightRear_ENU_interp - vehiclePose_ENU(:,1:3);
% 
% figure(343434)
% plot3(Temp_GPS(:,1),Temp_GPS(:,2),Temp_GPS(:,3),'b-','Linewidth',50);
% hold on
% grid on
% box on
% plot3(Left_GPS(:,1),Left_GPS(:,2),Left_GPS(:,3),'r-','Linewidth',50);
% plot3(Right_GPS(:,1),Right_GPS(:,2),Right_GPS(:,3),'k-','Linewidth',50);
% 
% xlabel('xEast', 'FontSize', 15);
% ylabel('yNorth', 'FontSize', 15);
% zlabel('zUp', 'FontSize', 15);


run('Updated_VehicleParameters_and_SensorParameters.m')

% The points in ENU coordinates.
sensorReading_ENU = GPS_SparkFun_RightRear_ENU_interp;

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_Left_Rear';

% GPS_SparkFun_Left_Rear
% Lidar_Velodyne_Rear

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);

% 1.1311    1.9333    1.3164
% -0.3548    1.9333    1.3164
