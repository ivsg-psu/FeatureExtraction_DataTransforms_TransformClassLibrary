function V_traj_unit = fcn_Calibration_CalculateGPSTrajectory(GPS_ENU_data)
% 
% [GPS_SparkFun_Front_ENU_interp, GPS_SparkFun_LeftRear_ENU_interp,GPS_SparkFun_RightRear_ENU_interp,TimeAligned] = fcn_GPSDataPreprocess(rawdata);
% GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU_interp(process_range,:);
% GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU_interp(process_range,:);
% GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU_interp(process_range,:);
% traj_range = 1:(length(process_range)+1);

V_traj = diff(GPS_ENU_data,1,1);
V_traj_rm = rmoutliers(V_traj);
V_traj_rm_mag = vecnorm(V_traj_rm,2,2);
V_traj_unit = V_traj_rm./V_traj_rm_mag;

% 
% V_left_traj = diff(GPS_SparkFun_LeftRear_ENU_interp(traj_range,:),1,1);
% V_left_traj = rmoutliers(V_left_traj);
% V_left_traj_mag = vecnorm(V_left_traj,2,2);
% V_left_traj_unit = V_left_traj./V_left_traj_mag;
% 
% V_right_traj = diff(GPS_SparkFun_RightRear_ENU_interp(traj_range,:),1,1);
% V_right_traj = rmoutliers(V_right_traj);
% V_right_traj_mag = vecnorm(V_right_traj,2,2);
% V_right_traj_unit = V_right_traj./V_right_traj_mag;


