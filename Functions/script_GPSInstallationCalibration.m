%% Load data from 10-13 for lane extraction
% clear all
date = "2023-10-13";
% bagFolderName_2 = "mapping_van_2023-09-29-14-25-13_0";
% bagFolderName_2 = "mapping_van_2023-10-09-NorthEntire";
% bagFolderName_2 = "mapping_van_2023-10-13-Detour";
bagFolderName_1 = "mapping_van_2023-10-13-16-48-01_0";
rawdata_1 = fcn_DataClean_loadMappingVanDataFromFile(bagFolderName_1,date,1);
bagFolderName_2 = "mapping_van_2023-10-13-16-50-00_0";
rawdata_2 = fcn_DataClean_loadMappingVanDataFromFile(bagFolderName_2,date,1);
bagFolderName_3 = "mapping_van_2023-10-13-16-51-23_0";
rawdata_3 = fcn_DataClean_loadMappingVanDataFromFile(bagFolderName_3,date,1);
bagFolderName_4 = "mapping_van_2023-10-13-16-53-04_0";
rawdata_4 = fcn_DataClean_loadMappingVanDataFromFile(bagFolderName_4,date,1);
bagFolderName_5 = "mapping_van_2023-10-13-16-55-47_0";
rawdata_5 = fcn_DataClean_loadMappingVanDataFromFile(bagFolderName_5,date,1);
bagFolderName_6 = "mapping_van_2023-10-13-16-57-53_0";
rawdata_6 = fcn_DataClean_loadMappingVanDataFromFile(bagFolderName_6,date,1);
%%
process_range = 1:1000;
figure(1)
clf
[V_front_traj_unit_1,V_left_traj_unit_1,V_right_traj_unit_1] = fcn_Calibration_CalculateGPSTrajectory(rawdata_1,process_range);
[V_front_traj_unit_2,V_left_traj_unit_2,V_right_traj_unit_2] = fcn_Calibration_CalculateGPSTrajectory(rawdata_2,process_range);
V_x_Estimation = mean([V_front_traj_unit_1;V_left_traj_unit_1;V_right_traj_unit_1]);
x_front_1 = zeros(size(V_right_traj_unit_1(:,1)));
y_front_1 = zeros(size(V_right_traj_unit_1(:,2)));
z_front_1 = zeros(size(V_right_traj_unit_1(:,3)));
quiver3(x_front_1,y_front_1,z_front_1,V_right_traj_unit_1(:,1),V_right_traj_unit_1(:,2),V_right_traj_unit_1(:,3),'Color','blue')
hold on
quiver3(0,0,0,V_x_Estimation(1),V_x_Estimation(2),V_x_Estimation(3),'Color','red','LineWidth',10)
% V_front_traj_unit_1_clean = rmoutliers(V_front_traj_unit_1);
% x_front_clean_1 = zeros(size(V_front_traj_unit_1_clean(:,1)));
% y_front_clean_1 = zeros(size(V_front_traj_unit_1_clean(:,2)));
% z_front_clean_1 = zeros(size(V_front_traj_unit_1_clean(:,3)));
% quiver3(x_front_clean_1,y_front_clean_1,z_front_clean_1,V_front_traj_unit_1_clean(:,1),V_front_traj_unit_1_clean(:,2),V_front_traj_unit_1_clean(:,3),'Color','red')
%%
process_range = 1:1000;
[yaw_offset_1, pitch_offset_1, ~, ~] = fcn_Transform_GPSInstallationCalibration(rawdata_1,process_range);
[yaw_offset_2, pitch_offset_2,~,~] = fcn_Transform_GPSInstallationCalibration(rawdata_2,process_range);
[yaw_offset_3, pitch_offset_3,~,~] = fcn_Transform_GPSInstallationCalibration(rawdata_3,process_range);
[yaw_offset_4, pitch_offset_4,~,~] = fcn_Transform_GPSInstallationCalibration(rawdata_4,process_range);
[yaw_offset_5, pitch_offset_5,~,~] = fcn_Transform_GPSInstallationCalibration(rawdata_5,process_range);
[yaw_offset_6, pitch_offset_6,~,~] = fcn_Transform_GPSInstallationCalibration(rawdata_6,process_range);
yaw_offset_calibrate = mean([yaw_offset_1,yaw_offset_2,yaw_offset_3,yaw_offset_4,yaw_offset_5,yaw_offset_6]);
pitch_offset_calibrate = mean([pitch_offset_1,pitch_offset_2,pitch_offset_3,pitch_offset_4,pitch_offset_5,pitch_offset_6]);
%%
calibrate_matrix = rotz(rad2deg(yaw_offset_calibrate))*roty(rad2deg(pitch_offset_calibrate));
save('GPS_Calibration_Matrix.mat','calibrate_matrix')