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
% 2024_11_04: Xinyu Cao
% -- wrote the script originally
%% NOTE: READ BEFORE RUNNING THE SCRIPT
% This script is a temporary *demo* illustrating Xinyu's point-cloud map-generation
% pipeline. It is not a finalized or fully released workflow.
%
% Several functions used here are still under development, and some utilities
% belong to the internal DataClean library. Because the DataClean library is
% not yet released, users must manually add it to the MATLAB path before
% running this demo.
%
% A pre-loaded dataset has been uploaded to:
%   F:\OneDrive - The Pennsylvania State University\IVSG\GitHubMirror\
%   MappingVanDataCollection\MatlabData\BaseMap\2025-09-30
% Its corresponding raw bag files can be found in:
%   F:\OneDrive - The Pennsylvania State University\IVSG\GitHubMirror\
%   MappingVanDataCollection\Bag Files\BaseMap\2025-09-30
% User can directly download them for test
% If any function does not work as expected, or if you encounter missing
% dependencies, please contact Xinyu at: xfc5113@psu.edu


%% Prep workspace
clc % Clear the console
close all % Close all figures

% _v2025_11_06';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/archive/refs/tags/DebugTools_v2025_11_06.zip';


ith_library = ith_library+1;
library_name{ith_library}    = 'PathClass_v2025_08_03';
library_folders{ith_library} = {'Functions'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary/archive/refs/tags/PathClass_v2025_08_03.zip';


ith_library = ith_library+1;
library_name{ith_library}    = 'LineFitting_v2023_07_24';
library_folders{ith_library} = {'Functions'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/FeatureExtraction_Association_LineFitting/archive/refs/tags/LineFitting_v2023_07_24.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'FindCircleRadius_v2023_08_02';
library_folders{ith_library} = {'Functions'};                                
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_FindCircleRadius/archive/refs/tags/FindCircleRadius_v2023_08_02.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'BreakDataIntoLaps_v2025_07_05';
library_folders{ith_library} = {'Functions'};                                
library_url{ith_library}     = 'https://github.com/ivsg-psu/FeatureExtraction_DataClean_BreakDataIntoLaps/archive/refs/tags/BreakDataIntoLaps_v2025_07_05.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'GeometryClass_v2025_10_20';
library_folders{ith_library} = {'Functions'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_GeomClassLibrary/archive/refs/tags/GeometryClass_v2025_10_20.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'PlotRoad_v2025_07_16';
library_folders{ith_library} = {'Functions'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/FieldDataCollection_VisualizingFieldData_PlotRoad/archive/refs/tags/PlotRoad_v2025_07_16.zip';

%% Clear paths and folders, if needed
clear flag_Transform_Folders_Initialized
fcn_INTERNAL_clearUtilitiesFromPathAndFolders;

%% Do we need to set up the work space?
if ~exist('flag_Transform_Folders_Initialized','var')
    this_project_folders = {'Functions','Data'};
    fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders);
    flag_Laps_Folders_Initialized = 1;
end

%% Set environment flags
setenv('MATLABFLAG_TRANSFORM_FLAG_CHECK_INPUTS','1');
setenv('MATLABFLAG_TRANSFORM_FLAG_DO_DEBUG','0');
%% Start of Demo Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   _____ _             _            __   _____                          _____          _
%  / ____| |           | |          / _| |  __ \                        / ____|        | |
% | (___ | |_ __ _ _ __| |_    ___ | |_  | |  | | ___ _ __ ___   ___   | |     ___   __| | ___
%  \___ \| __/ _` | '__| __|  / _ \|  _| | |  | |/ _ \ '_ ` _ \ / _ \  | |    / _ \ / _` |/ _ \
%  ____) | || (_| | |  | |_  | (_) | |   | |__| |  __/ | | | | | (_) | | |___| (_) | (_| |  __/
% |_____/ \__\__,_|_|   \__|  \___/|_|   |_____/ \___|_| |_| |_|\___/   \_____\___/ \__,_|\___|
%
%
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Start%20of%20Demo%20Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%
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


