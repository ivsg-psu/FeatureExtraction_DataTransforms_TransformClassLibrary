%% Introduction to the Data Transform Functions
%
% This is a demonstration script to show the primary functionality of the
% Data Transform library
%
% This script is to demonstrate the functions within the Data Transforms
% library. This code repo is typically located at:
% 
% https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_TransformClassLibrary
%
% If you have questions or comments, please contact Xinyu Cao: xfc5113@psu.edu
% Revision history:
%     
% 2024_11_05: Xinyu Cao
% -- wrote the script originally
% 2024_11_06: Xinyu Cao
% -- add NOTE 

%% NOTE: READ BEFORE RUNNING THE SCRIPT
% A test data data structure has been uploaded to 
% IVSG\GitHubMirror\MappingVanDataCollection\MatlabData\TestTrackOneLoop
% Please download and copy the TestTrackOneLoop folder to the LargeData
% Folder (need to be created by the user), then you can skip the loading
% and data clean process, start from line 112 after setup dependencies

% Same to other test scripts, no loading and data cleanning process needed
% if the pre-cleaned matfile has already been loaded

%% Prep workspace
clc % Clear the console
close all % Close all figures

%% Dependencies and Setup of the Code
% The code requires several other libraries to work, namely the following
%
% * DebugTools - the repo can be found at: https://github.com/ivsg-psu/Errata_Tutorials_DebugTools
% * GPS - this is the library that converts from ENU to/from LLA
% * DataClean - the repo can be found at: https://github.com/ivsg-psu/FeatureExtraction_DataCleanClassLibrary
% * 
% List what libraries we need, and where to find the codes for each
clear library_name library_folders library_url

ith_library = 1;
library_name{ith_library}    = 'DebugTools_v2024_10_17';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/archive/refs/tags/DebugTools_v2024_10_17.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'PathClass_v2024_03_14';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary/archive/refs/tags/PathClass_v2024_03_14.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'GeometryClass_v2024_08_28';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_GeomClassLibrary/archive/refs/tags/GeometryClass_v2024_08_28.zip';
%% Clear paths and folders, if needed
if 1==0
    clear flag_GeomClass_Folders_Initialized;
    fcn_INTERNAL_clearUtilitiesFromPathAndFolders;
end

%% Do we need to set up the work space?
if ~exist('flag_GeomClass_Folders_Initialized','var')
    this_project_folders = {'Functions','Data','LargeData'};
    fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders);
    flag_GeomClass_Folders_Initialized = 1;
end

%% Set environment flags for input checking
% These are values to set if we want to check inputs or do debugging
% setenv('MATLABFLAG_GEOMETRY_FLAG_CHECK_INPUTS','1');
% setenv('MATLABFLAG_GEOMETRY_FLAG_DO_DEBUG','1');
setenv('MATLABFLAG_GEOMETRY_FLAG_CHECK_INPUTS','1');
setenv('MATLABFLAG_GEOMETRY_FLAG_DO_DEBUG','0');
%% Define Flags and Identifiers
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

%% Load data
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

%% Load pre-cleaned mat file if you have downloaded the file
load("LargeData\TestTrackOneLoop\test_dataStructure.mat")
%% Grab ENU data from GPS units
[GPS_SparkFun_Front_ENU_array, GPS_SparkFun_LeftRear_ENU_array, GPS_SparkFun_RightRear_ENU_array] = fun_DataClean_extractENUCoordinatesFromGPS(filled_dataStructure);
% M_transform_RearRightGPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(GPSFront_ENU,GPSLeft_ENU, GPSRight_ENU, varargin);
%% Estimate Vehicle Pose
[VehiclePose,M_transform_Vehicle_to_ENU_matrix] = fcn_Transform_estimateVehiclePoseinENU(GPS_SparkFun_Front_ENU_array, GPS_SparkFun_LeftRear_ENU_array, GPS_SparkFun_RightRear_ENU_array);
%% Transform LiDAR data to ENU coordinate system
LiDAR_PointCloud_Transformed_Cell = fcn_Transform_transformLiDARToENU(filled_dataStructure);


