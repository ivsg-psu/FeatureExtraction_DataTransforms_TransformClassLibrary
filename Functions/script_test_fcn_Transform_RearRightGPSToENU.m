%% script_test_fcn_Transform_RearRightGPSToENU
% This test script is used to test fcn_Transform_CalculateTransformation_RearRightGPSToENU
% Revision history
% 2024_11_05 - Xinyu Cao, xfc5113@psu.edu
% -- wrote the code originally
%% Define prameters and Flags
Flags.flag_do_load_SICK = 0;
Flags.flag_do_load_Velodyne = 1;
Flags.flag_do_load_cameras = 0;
Flags.flag_do_load_all_data = 1;
Flags.flag_select_scan_duration = 0;
Flags.flag_do_load_GST = 0;
Flags.flag_do_load_VTG = 0;

clear Identifiers
Identifiers.Project = 'PennDOT ADS Workzones'; % This is the project sponsoring the data collection
Identifiers.ProjectStage = 'TestTrack'; % Can be 'Simulation', 'TestTrack', or 'OnRoad'
Identifiers.WorkZoneScenario = 'I376ParkwayPitt'; % Can be one of the ~20 scenarios, see key
Identifiers.WorkZoneDescriptor = 'WorkInRightLaneOfUndividedHighway'; % Can be one of the 20 descriptors, see key
Identifiers.Treatment = 'BaseMap'; % Can be one of 9 options, see key
Identifiers.DataSource = 'MappingVan'; % Can be 'MappingVan', 'AV', 'CV2X', etc. see key
Identifiers.AggregationType = 'PreRun'; % Can be 'PreCalibration', 'PreRun', 'Run', 'PostRun', or 'PostCalibration'
Identifiers.SourceBagFileName =''; % This is filled in automatically for each file

%% Load static LiDAR scan
fid = 1;
rootdirs{1} = fullfile(cd,'LargeData','2024-10-21','One Loop');
bagQueryString = 'mapping_van_2024-10-2*';
OneLoopCell = fcn_DataClean_loadRawDataFromDirectories(rootdirs, Identifiers,bagQueryString, fid,Flags);


%% Apply Data Clean LiDAR scan
num_datasets = length(OneLoopCell);
clean_dataset = {};
CleanOneLoopCell = [];
ref_basestation_TestTrack = [40.86368573 -77.83592832 344.189];
fid = 1;
for idx_dataset = 1:1
    rawdata_current = OneLoopCell{idx_dataset};
    cleanDataStruct = fcn_DataClean_cleanData(rawdata_current,ref_basestation_TestTrack,fid);
    converted_dataStructure = fcn_Transform_convertLLA2ENU(cleanDataStruct);
    filled_dataStructure = fcn_DataClean_fillGPSMissingData(converted_dataStructure);
    CleanOneLoopCell{idx_dataset,1} = filled_dataStructure;
    % clean_dataset{idx_dataset} = clean_rawdata;
end
%% Grab ENU data from GPS units
[GPS_SparkFun_Front_ENU_array, GPS_SparkFun_LeftRear_ENU_array, GPS_SparkFun_RightRear_ENU_array] = fun_DataClean_extractENUCoordinatesFromGPS(filled_dataStructure);
% M_transform_RearRightGPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU, varargin);
GPSFront_ENU = GPS_SparkFun_Front_ENU_array(end,:);
GPSLeft_ENU = GPS_SparkFun_LeftRear_ENU_array(end,:);
GPSRight_ENU = GPS_SparkFun_RightRear_ENU_array(end,:);
%% Test Case 1 - No plot
M_transform_RearRightGPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU);
%% Test Case 2 -Plot for debug
M_transform_RearRightGPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU,1,4);