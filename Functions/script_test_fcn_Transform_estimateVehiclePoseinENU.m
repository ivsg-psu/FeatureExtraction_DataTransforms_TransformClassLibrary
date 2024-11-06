% script_test_fcn_estimateVehiclePoseinENU.m
% tests fcn_estimateVehiclePoseinENU.m

% Revision history
% 2023_06_29 - Aneesh Batchu
% -- wrote the code originally
% 2024_11_05 - Xinyu Cao
% -- rewrite the code, rename the code from
% script_test_fcn_findVehiclePoseinENU to script_test_fcn_estimateVehiclePoseinENU

%% Set up the workspace
clc
close all

%% Check assertions for basic path operations and function testing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              _   _                 
%      /\                     | | (_)                
%     /  \   ___ ___  ___ _ __| |_ _  ___  _ __  ___ 
%    / /\ \ / __/ __|/ _ \ '__| __| |/ _ \| '_ \/ __|
%   / ____ \\__ \__ \  __/ |  | |_| | (_) | | | \__ \
%  /_/    \_\___/___/\___|_|   \__|_|\___/|_| |_|___/
%                                                    
%                                                    
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Assertions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
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
    filled_dataStructure = fcn_DataClean_fillMissingGPSENU(converted_dataStructure);
    CleanOneLoopCell{idx_dataset,1} = filled_dataStructure;
    % clean_dataset{idx_dataset} = clean_rawdata;
end
%% Grab ENU data from GPS units
[GPS_SparkFun_Front_ENU_array, GPS_SparkFun_LeftRear_ENU_array, GPS_SparkFun_RightRear_ENU_array] = fun_Transform_extractENUCoordinatesFromGPS(filled_dataStructure);
% M_transform_RearRightGPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU, varargin);

%% Test Case 1 - No plot
[VehiclePose,M_transform_Vehicle_to_ENU_matrix] = fcn_Transform_estimateVehiclePoseinENU(GPS_SparkFun_Front_ENU_array, GPS_SparkFun_LeftRear_ENU_array, GPS_SparkFun_RightRear_ENU_array);
%% Test Case 2 -Plot for debug
fig_num = 104;
fid = 1;
[VehiclePose,M_transform_Vehicle_to_ENU_matrix] = fcn_Transform_estimateVehiclePoseinENU(GPS_SparkFun_Front_ENU_array, GPS_SparkFun_LeftRear_ENU_array, GPS_SparkFun_RightRear_ENU_array,[],[],fid,fig_num);