function [GPS_Data_Struct_Output,traj_range] = fcn_GPS_Calibration_selectProcessRange(GPS_Data_Struct, process_range)

GPS_SparkFun_Front_ENU = GPS_Data_Struct.GPS_SparkFun_Front_ENU;
GPS_SparkFun_LeftRear_ENU = GPS_Data_Struct.GPS_SparkFun_LeftRear_ENU;
GPS_SparkFun_RightRear_ENU = GPS_Data_Struct.GPS_SparkFun_RightRear_ENU;


if process_range == 'all'
    GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU;
    GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU;
    GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU;
    traj_range = 1:(length(GPS_SparkFun_Front_ENU_selected)+1);
else
    GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU(process_range,:);
    GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU(process_range,:);
    GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU(process_range,:);
    traj_range = 1:(length(process_range)+1);
end

GPS_Data_Struct_Output = struct;
GPS_Data_Struct_Output.GPS_SparkFun_Front_ENU = GPS_SparkFun_Front_ENU_selected;
GPS_Data_Struct_Output.GPS_SparkFun_LeftRear_ENU = GPS_SparkFun_LeftRear_ENU_selected;
GPS_Data_Struct_Output.GPS_SparkFun_RightRear_ENU = GPS_SparkFun_RightRear_ENU_selected;