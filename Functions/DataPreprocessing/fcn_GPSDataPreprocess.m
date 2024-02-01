function [GPS_SparkFun_Front_ENU_interp,...
        GPS_SparkFun_LeftRear_ENU_interp,....
        GPS_SparkFun_RightRear_ENU_interp,...
        GPS_SparkFun_Temp_ENU_interp,...
        Lidar_Sick_Rear_Valid,...
        Lidar_Velodyne_Rear_Valid,...
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

if any(contains(fields,'GPS_SparkFun_Temp_GGA'))
    GPS_SparkFun_Temp_GGA = rawdata.GPS_SparkFun_Temp_GGA;
    GPS_SparkFun_Temp_GGA_Locked = fcn_DataPreprocessing_RemoveUnlockedData(GPS_SparkFun_Temp_GGA);
else
    GPS_SparkFun_Temp_GGA_Locked = [];  
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
rawData_Locked.GPS_SparkFun_Temp_GGA_Locked = GPS_SparkFun_Temp_GGA_Locked;
rawData_Locked.Lidar_Velodyne_Rear = Lidar_Velodyne_Rear;
rawData_Locked.Lidar_Sick_Rear = Lidar_Sick_Rear;

%% Step 2: Trim the time, determine the valid time range for all the seneors 
time_range = fcn_DataPreprocessing_FindMaxAndMinTime(rawData_Locked);
GPS_SparkFun_LeftRear_GGA_Valid = fcn_DataPreprocessing_SelectValidData(GPS_SparkFun_LeftRear_GGA_Locked,time_range,'gps');
GPS_SparkFun_RightRear_GGA_Valid = fcn_DataPreprocessing_SelectValidData(GPS_SparkFun_RightRear_GGA_Locked,time_range,'gps');
GPS_SparkFun_Front_GGA_Valid = fcn_DataPreprocessing_SelectValidData(GPS_SparkFun_Front_GGA_Locked,time_range,'gps');
GPS_SparkFun_Temp_GGA_Valid = fcn_DataPreprocessing_SelectValidData(GPS_SparkFun_Temp_GGA_Locked,time_range,'gps');
Lidar_Sick_Rear_Valid = fcn_DataPreprocessing_SelectValidData(Lidar_Sick_Rear,time_range,'lidar');
Lidar_Velodyne_Rear_Valid = fcn_DataPreprocessing_SelectValidData(Lidar_Velodyne_Rear,time_range,'lidar');

%% Step 3: Interpolate data 
if strcmpi(LidarType,'sick')
    fq_Lidar = 25;
    Npoints_Lidar = Lidar_Sick_Rear_Valid.Npoints;

elseif strcmpi(LidarType,'velodyne')
    fq_Lidar = 10;
    Npoints_Lidar = Lidar_Velodyne_Rear_Valid.Npoints;
    
end

dt = 1/fq_Lidar;
EndTime = Npoints_Lidar*dt;
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

if ~isempty(GPS_SparkFun_Temp_GGA_Valid)
    GPS_SparkFun_Temp_LLA = [GPS_SparkFun_Temp_GGA_Valid.Latitude,GPS_SparkFun_Temp_GGA_Valid.Longitude,GPS_SparkFun_Temp_GGA_Valid.Altitude];
    GPS_SparkFun_Temp_ENU = lla2enu(GPS_SparkFun_Temp_LLA,ref_basestation,'ellipsoid');
    GPS_Temp_GPSTime = GPS_SparkFun_Temp_GGA_Valid.GPS_Time - GPS_SparkFun_Temp_GGA_Valid.GPS_Time(1);
    GPS_SparkFun_Temp_ENU_interp = interp1(GPS_Temp_GPSTime,GPS_SparkFun_Temp_ENU,TimeAligned);
end




