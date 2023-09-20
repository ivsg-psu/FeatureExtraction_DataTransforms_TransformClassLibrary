
% run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')
%%

load PoseData_1.mat

LeftGPS_ENU = GPS_SparkFun_LeftRear_ENU_interp ;

RightGPS_ENU = GPS_SparkFun_RightRear_ENU_interp;

PITCH_vehicle_ENU = zeros(size(LeftGPS_ENU,1),1);

figure(343434)
plot3(LeftGPS_ENU(:,1),LeftGPS_ENU(:,2),LeftGPS_ENU(:,3),'b-','Linewidth',3);
hold on
grid on
box on
plot3(RightGPS_ENU(:,1),RightGPS_ENU(:,2),RightGPS_ENU(:,3),'r-','Linewidth',3);


% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center] in meters
SensorMount_offset_relative_to_VehicleOrigin = [-0.6027 0 1.5261]; 

vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(LeftGPS_ENU, RightGPS_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);


vehiclePose_ENU_pos = vehiclePose_ENU(:,1:3);

plot3(vehiclePose_ENU_pos(:,1),vehiclePose_ENU_pos(:,2),vehiclePose_ENU_pos(:,3),'k-','Linewidth',3);


% Left_GPS = GPS_SparkFun_LeftRear_ENU_interp - vehiclePose_ENU_pos;
% Right_GPS = GPS_SparkFun_RightRear_ENU_interp - vehiclePose_ENU_pos;
% 
% plot3(Left_GPS(:,1),Left_GPS(:,2),Left_GPS(:,3),'r-','Linewidth',50);
% plot3(Right_GPS(:,1),Right_GPS(:,2),Right_GPS(:,3),'k-','Linewidth',50);


% % Repeat the SensorMount_offset_relative_to_VehicleOrigin 
% SensorMount_offset_relative_to_VehicleOrigin_repeat = repmat(SensorMount_offset_relative_to_VehicleOrigin, size(LeftGPS_ENU,1), 1);
% 
% GPS_Mount_Origin = vehiclePose_ENU_pos + SensorMount_offset_relative_to_VehicleOrigin_repeat;

plot3(GPS_Mount_Origin(:,1),GPS_Mount_Origin(:,2),GPS_Mount_Origin(:,3),'m-','Linewidth',3);


%% Bug's Fixed - Load PoseData.mat


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
sensorReading_ENU = GPS_SparkFun_LeftRear_ENU_interp;

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';

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
