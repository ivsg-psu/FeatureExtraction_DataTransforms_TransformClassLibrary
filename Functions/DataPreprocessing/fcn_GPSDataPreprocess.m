function [GPS_Data_Struct_Output,...
        Lidar_Sick_Rear_Output,...
        Lidar_Velodyne_Rear_Output,...
        TimeAligned] = fcn_GPSDataPreprocess(rawdata,LidarType,ref_basestation)


%% Step 1 - Remove unlocked GPS data for all three GPS structs
fields = fieldnames(rawdata);
if any(contains(fields,'GPS_SparkFun_LeftRear_GGA'))
    GPS_SparkFun_LeftRear_GGA = rawdata.GPS_SparkFun_LeftRear_GGA;
    GPS_SparkFun_LeftRear_GGA_Locked = fcn_DataPreprocessing_RemoveUnlockedData(GPS_SparkFun_LeftRear_GGA);
else
    GPS_SparkFun_LeftRear_GGA_Locked = [];
end

if any(contains(fields,'GPS_SparkFun_RightRear_GGA'))
    GPS_SparkFun_RightRear_GGA = rawdata.GPS_SparkFun_RightRear_GGA;
    GPS_SparkFun_RightRear_GGA_Locked = fcn_DataPreprocessing_RemoveUnlockedData(GPS_SparkFun_RightRear_GGA);
else
    GPS_SparkFun_RightRear_GGA_Locked = [];
end

if any(contains(fields,'GPS_SparkFun_Front_GGA'))
    GPS_SparkFun_Front_GGA = rawdata.GPS_SparkFun_Front_GGA;
    GPS_SparkFun_Front_GGA_Locked = fcn_DataPreprocessing_RemoveUnlockedData(GPS_SparkFun_Front_GGA);
else
    GPS_SparkFun_Front_GGA_Locked = [];
end

if any(contains(fields,'Lidar_Velodyne_Rear'))
    Lidar_Velodyne_Rear = rawdata.Lidar_Velodyne_Rear;
else
    Lidar_Velodyne_Rear = [];
end
if any(contains(fields,'Lidar_Sick_Rear'))
    Lidar_Sick_Rear = rawdata.Lidar_Sick_Rear;
else
    Lidar_Sick_Rear = [];
end

rawData_Locked = struct;
rawData_Locked.GPS_SparkFun_LeftRear_GGA_Locked = GPS_SparkFun_LeftRear_GGA_Locked;
rawData_Locked.GPS_SparkFun_RightRear_GGA_Locked = GPS_SparkFun_RightRear_GGA_Locked;
rawData_Locked.GPS_SparkFun_Front_GGA_Locked = GPS_SparkFun_Front_GGA_Locked;
rawData_Locked.Lidar_Velodyne_Rear = Lidar_Velodyne_Rear;
rawData_Locked.Lidar_Sick_Rear = Lidar_Sick_Rear;

%% Step 2: Trim the time, determine the valid time range for all the seneors 
time_range = fcn_DataPreprocessing_FindMaxAndMinTime(rawData_Locked);
GPS_SparkFun_LeftRear_GGA_Valid = fcn_DataPreprocessing_SelectValidData(GPS_SparkFun_LeftRear_GGA_Locked,time_range,'gps');
GPS_SparkFun_RightRear_GGA_Valid = fcn_DataPreprocessing_SelectValidData(GPS_SparkFun_RightRear_GGA_Locked,time_range,'gps');
GPS_SparkFun_Front_GGA_Valid = fcn_DataPreprocessing_SelectValidData(GPS_SparkFun_Front_GGA_Locked,time_range,'gps');
Lidar_Sick_Rear_Valid = fcn_DataPreprocessing_SelectValidData(Lidar_Sick_Rear,time_range,'lidar');
Lidar_Velodyne_Rear_Valid = fcn_DataPreprocessing_SelectValidData(Lidar_Velodyne_Rear,time_range,'lidar');

%% Step 3: Interpolate data 
if strcmpi(LidarType,'sick')&(~isempty(Lidar_Sick_Rear))
    fq_interp = 25;
    Npoints_Lidar = Lidar_Sick_Rear_Valid.Npoints;
    Npoints_interp = Npoints_Lidar;
elseif strcmpi(LidarType,'velodyne')&(~isempty(Lidar_Velodyne_Rear))
    fq_interp = 10;
    Npoints_Lidar = Lidar_Velodyne_Rear_Valid.Npoints;
    Npoints_interp = Npoints_Lidar;
elseif strcmpi(LidarType,'none')
    fq_interp = 10;
    Npoints_interp = max([GPS_SparkFun_LeftRear_GGA_Valid.Npoints,GPS_SparkFun_RightRear_GGA_Valid.Npoints,GPS_SparkFun_Front_GGA_Valid.Npoints]);
end

dt = 1/fq_interp;
EndTime = Npoints_interp*dt;
TimeAligned = (dt:dt:EndTime).';

if ~isempty(GPS_SparkFun_Front_GGA_Valid)
    GPS_SparkFun_Front_LLA = [GPS_SparkFun_Front_GGA_Valid.Latitude,GPS_SparkFun_Front_GGA_Valid.Longitude,GPS_SparkFun_Front_GGA_Valid.Altitude];
    GPS_SparkFun_Front_ENU = lla2enu(GPS_SparkFun_Front_LLA,ref_basestation,'ellipsoid');
    GPS_Front_GPSTime = GPS_SparkFun_Front_GGA_Valid.GPS_Time - GPS_SparkFun_Front_GGA_Valid.GPS_Time(1);
    GPS_SparkFun_Front_ENU_interp = interp1(GPS_Front_GPSTime,GPS_SparkFun_Front_ENU,TimeAligned);
else
    GPS_SparkFun_Front_ENU_interp = [];
end
if ~isempty(GPS_SparkFun_LeftRear_GGA_Valid)

    GPS_SparkFun_LeftRear_LLA = [GPS_SparkFun_LeftRear_GGA_Valid.Latitude,GPS_SparkFun_LeftRear_GGA_Valid.Longitude,GPS_SparkFun_LeftRear_GGA_Valid.Altitude];
    GPS_SparkFun_LeftRear_ENU = lla2enu(GPS_SparkFun_LeftRear_LLA,ref_basestation,'ellipsoid');
    GPS_LeftRear_GPSTime = GPS_SparkFun_LeftRear_GGA_Valid.GPS_Time - GPS_SparkFun_LeftRear_GGA_Valid.GPS_Time(1);
    GPS_SparkFun_LeftRear_ENU_interp = interp1(GPS_LeftRear_GPSTime,GPS_SparkFun_LeftRear_ENU,TimeAligned);
end

if ~isempty(GPS_SparkFun_RightRear_GGA_Valid)
    GPS_SparkFun_RightRear_LLA = [GPS_SparkFun_RightRear_GGA_Valid.Latitude,GPS_SparkFun_RightRear_GGA_Valid.Longitude,GPS_SparkFun_RightRear_GGA_Valid.Altitude];
    GPS_SparkFun_RightRear_ENU = lla2enu(GPS_SparkFun_RightRear_LLA,ref_basestation,'ellipsoid');
    GPS_RightRear_GPSTime = GPS_SparkFun_RightRear_GGA_Valid.GPS_Time - GPS_SparkFun_RightRear_GGA_Valid.GPS_Time(1);
    GPS_SparkFun_RightRear_ENU_interp = interp1(GPS_RightRear_GPSTime,GPS_SparkFun_RightRear_ENU,TimeAligned);
end

%% Remove nan data
tf_nonnan_GPS_Left = ~isnan(GPS_SparkFun_LeftRear_ENU_interp(:,1));
tf_nonnan_GPS_Right = ~isnan(GPS_SparkFun_RightRear_ENU_interp(:,1));
tf_nonnan_GPS_Front = ~isnan(GPS_SparkFun_Front_ENU_interp(:,1));
tf_nonnan = tf_nonnan_GPS_Left&tf_nonnan_GPS_Right&tf_nonnan_GPS_Front;
idxs_nonnan = find(tf_nonnan);
GPS_SparkFun_LeftRear_ENU_output = GPS_SparkFun_LeftRear_ENU_interp(idxs_nonnan,:);
GPS_SparkFun_Front_ENU_output = GPS_SparkFun_Front_ENU_interp(idxs_nonnan,:);
GPS_SparkFun_RightRear_ENU_output = GPS_SparkFun_RightRear_ENU_interp(idxs_nonnan,:);
GPS_Data_Struct_Output = struct;
GPS_Data_Struct_Output.GPS_SparkFun_Front_ENU = GPS_SparkFun_Front_ENU_output;
GPS_Data_Struct_Output.GPS_SparkFun_LeftRear_ENU = GPS_SparkFun_LeftRear_ENU_output;
GPS_Data_Struct_Output.GPS_SparkFun_RightRear_ENU = GPS_SparkFun_RightRear_ENU_output;
Lidar_Sick_Rear_Output = Lidar_Sick_Rear_Valid;
Lidar_Velodyne_Rear_Output = Lidar_Velodyne_Rear_Valid;




