function XYZ_Lidar_ENU = fcn_Transform_SickLidarCoordToENU(GPSLeft_ENU, GPSRight_ENU, GPSFront_ENU,XYZ_Lidar, sensorPoseParameters,calibrate_matrix)


[roll,pitch,yaw] = fcn_Transform_CalculateAnglesofRotation(GPSLeft_ENU,GPSRight_ENU,GPSFront_ENU,calibrate_matrix);

%% Step 2 - Find the Position of the Sensor Mount and find the transormation matrix from vehicle origin to sensor mount

sensorMount_center = (GPSLeft_ENU+GPSRight_ENU)/2;
sensorMount_PoseENU = [sensorMount_center,roll,pitch,yaw];
N_points = size(sensorMount_PoseENU,1);

Lidar_Sick_Rear_offset_x_relative_to_sensorplatform = sensorPoseParameters.Lidar_Sick_Rear.offset_x_relative_to_sensorplatform; % Meters - GUESS!!
Lidar_Sick_Rear_offset_y_relative_to_sensorplatform = sensorPoseParameters.Lidar_Sick_Rear.offset_y_relative_to_sensorplatform; % Meters - GUESS!!
Lidar_Sick_Rear_offset_z_relative_to_sensorplatform = sensorPoseParameters.Lidar_Sick_Rear.offset_z_relative_to_sensorplatform; % Meters - GUESS!!

% Transform matrices to move the Lidar_Sick_Rear to its correct location
Mtransform_Lidar_Sick_Rear_translate = makehgtform('translate',[Lidar_Sick_Rear_offset_x_relative_to_sensorplatform, ...
                                                                Lidar_Sick_Rear_offset_y_relative_to_sensorplatform, ...
                                                                Lidar_Sick_Rear_offset_z_relative_to_sensorplatform]);

Mtransform_Lidar_Sick_Rear_zrotate = makehgtform('zrotate',deg2rad(sensorPoseParameters.Lidar_Sick_Rear.yaw_relative_to_own_axis));
Mtransform_Lidar_Sick_Rear_yrotate = makehgtform('yrotate',deg2rad(sensorPoseParameters.Lidar_Sick_Rear.pitch_relative_to_own_axis));
Mtransform_Lidar_Sick_Rear_xrotate = makehgtform('xrotate',deg2rad(sensorPoseParameters.Lidar_Sick_Rear.roll_relative_to_own_axis));
Mtransform_RearGPSCenter_to_SickLidar = Mtransform_Lidar_Sick_Rear_translate*Mtransform_Lidar_Sick_Rear_zrotate*Mtransform_Lidar_Sick_Rear_yrotate*Mtransform_Lidar_Sick_Rear_xrotate;
%% Step 3 - Calculate the Vehicle Position
for n = 1:N_points
    Mtransform_ENU_to_RearGPSCenter = fcn_Transform_CalculateTransformation_RearGPSCenter(sensorMount_PoseENU(n,:));
    Mtransform_VehicleOrigin_to_ENU = Mtransform_ENU_to_RearGPSCenter*Mtransform_RearGPSCenter_to_SickLidar;
    XYZ_n = XYZ_Lidar(n,:);
    XYZ_Homo = Mtransform_VehicleOrigin_to_ENU*[XYZ_n,1].';
    % VehiclePose_ENU_array(n,:) = VehiclePose_ENU_Homo(1:3).';
    XYZ_Lidar_ENU(n,:) = XYZ_Homo(1:3).';

end

fprintf(1,'\nThe POSE of the vehicle in ENU coordinates is :\n');
% disp(VehiclePose);