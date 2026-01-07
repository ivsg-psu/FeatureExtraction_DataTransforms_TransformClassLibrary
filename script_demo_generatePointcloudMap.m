%% Introduction to and Purpose of the Code
% This is the explanation of the code that can be found by running
%       script_demo_generatePointcloudMap.m
% This is a script to demonstrate how to generate pointcloud map
% This code builds on the demo structure of "script_demo_Laps.m", but is
% repurposed to demonstrate feature extraction instead of lap detection.
%
% Typical repo location:
%   https://github.com/ivsg-psu/FeatureExtraction_DataTransforms_TransformClassLibrary
%
% Contact: Xinyu Cao or Dr. Sean Brennan at sbrennan@psu.edu

%% Revision history:

% 2025_11_23 - Xinyu Cao
% -- Created based on Laps demo structure
% 2025_11_25 - Xinyu Cao
% -- Modified NOTE
%% NOTE: READ BEFORE RUNNING THE SCRIPT
% This script is a temporary *demo* illustrating Xinyu's point-cloud
% map-generation pipeline. It is not a finalized or publicly released workflow.
%
% Several functions used here are still under development, and some utilities
% belong to the internal DataClean library. Because the DataClean library is
% not yet released, users must manually add it to the MATLAB path before
% running this demo.
%
% -------------------------------------------------------------------------
% DATA SOURCES
% -------------------------------------------------------------------------
% A pre-loaded dataset has been uploaded to:
%   F:\OneDrive - The Pennsylvania State University\IVSG\GitHubMirror\
%   MappingVanDataCollection\MatlabData\BaseMap\2025-09-30
%
% Corresponding raw ROS bag files are stored in:
%   F:\OneDrive - The Pennsylvania State University\IVSG\GitHubMirror\
%   MappingVanDataCollection\Bag Files\BaseMap\2025-09-30
%
% Users can directly download these datasets and run the demo without
% additional processing.
%
% -------------------------------------------------------------------------
% IF STARTING FROM RAW BAG FILES
% -------------------------------------------------------------------------
% If you prefer to begin from the raw bag files instead of the pre-parsed
% MATLAB data, you *must* first parse the bags using:
%
%       script_main_ParseRaw
%
% This parsing step converts ROS bag files into the correct directory
% structure expected by the DataClean loader. Because parsing takes time,
% it is recommended to start with the **smallest bag file** for initial
% testing.
%
% After parsing, place the output folders under a path such as:
%
%   MappingVanData\TestTrack\BaseMap\2025_09_30\BaseMap_<yourName>_0930\
%
% Replace <yourName> with your own identifier.
%
% IMPORTANT:
%   - Update the variables `MappingVanDataRoot` and `rootdirs` in this demo
%     script so that they point to your newly parsed directory.
%   - The function `fcn_DataClean_loadRawDataFromDirectories` can **only**
%     load *parsed* raw data; it cannot ingest untouched ROS bag files.
%
% If any function does not work as expected, or if you encounter missing
% dependencies or path issues, please contact:
%
%       xfc5113@psu.edu
%
% -------------------------------------------------------------------------


%% Prepare the workspace
clear all
close all

clear library_name library_folders library_url

ith_library = 1;
library_name{ith_library}    = 'DebugTools_v2025_11_06';
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

disp('Welcome to the demo code for the ExtractCL library!')

addpath('C:\Users\abb6486\Documents\IVSG\FeatureExtraction_DataCleanClassLibrary\Functions\fcn_DataClean_loadRawData');
addpath('C:\Users\abb6486\Documents\IVSG\FeatureExtraction_DataCleanClassLibrary\Functions');

%% Define Flags and Identifiers
Flags.flag_do_load_SICK = 0;
Flags.flag_do_load_Velodyne = 1;
Flags.flag_do_load_cameras = 0;
Flags.flag_do_load_all_data = 1;
Flags.flag_select_scan_duration = 0;
Flags.flag_do_load_GST = 0;
Flags.flag_do_load_VTG = 0;
Flags.flag_do_load_PVT = 0;
Flags.flag_do_load_Ouster = 0;
clear Identifiers
Identifiers.Project = 'PennDOT ADS Workzones'; % This is the project sponsoring the data collection
Identifiers.ProjectStage = 'TestTrack'; % Can be 'Simulation', 'TestTrack', or 'OnRoad'
Identifiers.WorkZoneScenario = 'BaseMap'; % Can be one of the ~20 scenarios, see key
Identifiers.WorkZoneDescriptor = 'WorkInRightLaneOfUndividedHighway'; % Can be one of the 20 descriptors, see key
Identifiers.Treatment = 'BaseMap'; % Can be one of 9 options, see key
Identifiers.DataSource = 'MappingVan'; % Can be 'MappingVan', 'AV', 'CV2X', etc. see key
Identifiers.AggregationType = 'PreRun'; % Can be 'PreCalibration', 'PreRun', 'Run', 'PostRun', or 'PostCalibration'
Identifiers.SourceBagFileName =''; % This is filled in automatically for each file

%% Load data set
fid = 1;
clear rawDataCellArray
% Please change the root to your path, including the date and folder name
MappingVanDataRoot = "C:\MappingVanData\ParsedBags";

Location = Identifiers.ProjectStage;
Treatment = Identifiers.WorkZoneScenario;
Date = "2025-09-30";
rootdirs{1} = fullfile(MappingVanDataRoot, Location, Treatment,Date,"BaseMap_Aneesh_20250930");

bagQueryString = 'mapping_van_2025-09-30*';
rawDataCellArray = fcn_DataClean_loadRawDataFromDirectories(rootdirs, Identifiers,bagQueryString, fid,Flags);
assert(iscell(rawDataCellArray));

%% Load transformation matrices
load("Data\Config\M_calibration_GPS_to_Vehicle_2024-05-15.mat") % GPS calibration matrix
load("Data\Config\RearRightGPS_offset_relative_to_VehicleOrigin.mat") % Relative offset from rear right GPS to vehicle origin
load("Data\Config\M_transform_LiDARVelodyne_to_RearRightGPS_2025_03_27.mat") % Transformation from Velodyne LiDAR to rear right GPS
%% Merge datasets belonging to one collection
% Specify the nearby time
thresholdTimeNearby = 2;
% Spedify the fid
fid = 1; % 1 --> print to console

% List what will be saved
saveFlags.flag_saveMatFile = 1;
saveFlags.flag_saveMatFile_directory = fullfile(cd,'Data','RawDataMerged',Identifiers.ProjectStage,Identifiers.WorkZoneScenario);
saveFlags.flag_saveImages = 1;
saveFlags.flag_saveImages_directory  = fullfile(cd,'Data','RawDataMerged',Identifiers.ProjectStage,Identifiers.WorkZoneScenario);
saveFlags.flag_saveImages_name = cat(2,Identifiers.WorkZoneScenario,'_merged');
saveFlags.flag_forceDirectoryCreation = 1;
saveFlags.flag_forceImageOverwrite = 1;
saveFlags.flag_forceMATfileOverwrite = 1;

% List what will be plotted, and the figure numbers
plotFlags.fig_num_plotAllMergedTogether = 1111;
plotFlags.fig_num_plotAllMergedIndividually = 2222;
    
plotFlags.mergedplotFormat.LineStyle = '-';
plotFlags.mergedplotFormat.LineWidth = 2;
plotFlags.mergedplotFormat.Marker = 'none';
plotFlags.mergedplotFormat.MarkerSize = 5;

% Call the function
[mergedRawDataCellArray, uncommonFieldsCellArray] = fcn_DataClean_mergeRawDataStructures(rawDataCellArray, (thresholdTimeNearby), (fid), (saveFlags), (plotFlags));
%% Convert LLA to ENU and Fill the xEast, yNorth and zUp fields for GPS units
N_datasets = length(mergedRawDataCellArray);
ref_baseStationLLA = [40.86368573 -77.83592832 344.189]; % Test track base station
filledDataCellArray = cell(N_datasets,1);
fid = 1;
for idx_dataset = 1:N_datasets
    rawDataStructure = mergedRawDataCellArray{idx_dataset};
    filledDataStructure =  fcn_Transform_fillGPSENUFields(rawDataStructure,ref_baseStationLLA);
    filledDataCellArray{idx_dataset} = filledDataStructure;
end

%% Break data into laps
% Define start_definition
startline = [445.871882554 209.029823474;
            431.135590234 198.199778077];

%% Break GPS data into laps
% [lap_traversals_VehiclePose,~,~,laps_indices] = fcn_Laps_breakDataIntoLaps(VehiclePose_traversal,startline);
% laps_indices_cell = fcn_Laps_breakDataIntoLapIndices(VehiclePose_ENU,startline_straight);



%% Perform motion compensation
% This step is a key part of the point-cloud map generation pipeline: it
% compensates for vehicle motion during each LiDAR scan so that the
% accumulated map is as geometrically consistent as possible.
%
% However, if you only need to transform a single-frame point cloud from the
% LiDAR frame to the vehicle frame (without building a multi-scan map),
% motion compensation is not required. In that case, set:
%
%   flag_do_motion_compensation = 0;
flag_do_motion_compensation = 0;
N_datasets = length(filledDataCellArray);
algnedDataCellArray = cell(N_datasets,1);
lidar_fieldname = 'Lidar_Velodyne_Rear';
time_decimation = 0.005;
TimeField = 'Device_Time';
for idx_dataset = 1:N_datasets
    filledDataStructure = filledDataCellArray{idx_dataset};
    lockedDataStruct = fcn_DataClean_RemoveUnlockedData(filledDataStructure);
    % %% Trim the GPS data to the same length (Already in the dataClean funciton)
    [trimmedDataStruct, t0] = fcn_DataClean_TrimByCommonGPS(lockedDataStruct, ...
    'LiDARFieldName',lidar_fieldname, ...
    'GPSPrefix','GPS_', 'GPSTimeField','GPS_Time', ...
    'Inclusive',true, 'Debug',true,'TrimLiDAR', false);
    time_offset = 0;
    if flag_do_motion_compensation
        [deskewedDataStruct, Bins_timestamps] = fcn_DataClean_splitPointCloudByTime(trimmedDataStruct, time_offset, time_decimation,'TimeField',TimeField);
    else
        deskewedDataStruct = trimmedDataStruct;
    end
        alignedDataStructure = fcn_DataClean_synchronizeGPSToLiDARTime( ...
        deskewedDataStruct, lidar_fieldname,'TimeField',TimeField,'TimeOffset',time_offset);
    alignedDataCellArray{idx_dataset} = alignedDataStructure;
end
%% Perform transformation
field_name = 'GPS_Time';

fill_type = 1;
ROI = struct;
ROI.X_lim = [-10 10];
ROI.Y_lim = [-40 40];
ROI.Z_lim = [-5 10];
Flags.flag_skip_ahead_point = 1;
Flags.filter_in_XYZ = 1;
Flags.plot_raw_LIDAR = 0;
alignedDataENUCellArray = cell(N_datasets,1);
for idx_dataset = 1:1
    
    alignedDataStructure = alignedDataCellArray{idx_dataset};
    LiDAR_DataStructure = alignedDataStructure.Lidar_Velodyne_Rear;
    %% Transform LiDAR pointcloud from LiDAR frame to vehicle frame
    LiDAR_DataStructure_Vehicle = fcn_Transform_transformLiDARToVehicle(LiDAR_DataStructure,RearRightGPS_offset_relative_to_VehicleOrigin,...
        M_calibration_GPS_to_Vehicle, M_transform_LiDARVelodyne_to_RearRightGPS);
    %% Filter the LiDAR pointcloud in vehicle frame in the defined ROI
    LiDAR_DataStructure_Vehicle_filtered = fcn_Transform_filterPointCloudInROI(LiDAR_DataStructure_Vehicle,ROI);
    croppedDataStructure = alignedDataStructure;
    croppedDataStructure.Lidar_Velodyne_Rear = LiDAR_DataStructure_Vehicle_filtered;
    alignedDataStruct_ENU = croppedDataStructure;
    %% Transform LiDAR pointcloud from vehicle frame to ENU frame
    GPS_Front_Struct = croppedDataStructure.GPS_SparkFun_Front_GGA;
    GPS_Front_ENU_array = [GPS_Front_Struct.xEast, GPS_Front_Struct.yNorth, GPS_Front_Struct.zUp];
    GPS_LeftRear_Struct = croppedDataStructure.GPS_SparkFun_LeftRear_GGA;
    GPS_LeftRear_ENU_array = [GPS_LeftRear_Struct.xEast, GPS_LeftRear_Struct.yNorth, GPS_LeftRear_Struct.zUp];
    GPS_RightRear_Struct = croppedDataStructure.GPS_SparkFun_RightRear_GGA;

    GPS_RightRear_ENU_array = [GPS_RightRear_Struct.xEast, GPS_RightRear_Struct.yNorth, GPS_RightRear_Struct.zUp];
    [LiDAR_Velodyne_Rear_ENU,VehiclePose] = fcn_Transform_transformVehicleToENU(LiDAR_DataStructure_Vehicle_filtered,GPS_Front_ENU_array,...
        GPS_LeftRear_ENU_array, GPS_RightRear_ENU_array,RearRightGPS_offset_relative_to_VehicleOrigin,M_calibration_GPS_to_Vehicle);
    alignedDataStruct_ENU.Lidar_Velodyne_Rear = LiDAR_Velodyne_Rear_ENU;
    alignedDataStruct_ENU.VehiclePose = VehiclePose;
    alignedDataENUCellArray{idx_dataset} = alignedDataStruct_ENU;
end
%% Break into laps if needed
breakedCell = {};
for idx_dataset = 1:N_datasets
    alignedDataStruct_ENU = alignedDataENUCellArray{1};
    VehiclePose = alignedDataStruct_ENU.VehiclePose;
    Pointcloud_ENU_cell = alignedDataStruct_ENU.Lidar_Velodyne_Rear.PointCloud;
    rwoMask = all(isfinite(VehiclePose), 2);
    VehiclePose = VehiclePose(rwoMask,:);
    Pointcloud_ENU_cell = Pointcloud_ENU_cell(rwoMask,:);
    laps_indices_cell = fcn_Laps_breakDataIntoLapIndices(...
        VehiclePose(:,1:3),...
        startline);
    N_laps = length(laps_indices_cell);
    for idx_lap = 1:N_laps
        lap_indices = laps_indices_cell{idx_lap};
        Pointcloud_ENU_ith_lap = Pointcloud_ENU_cell(lap_indices,:);
        breakedCell{end + 1,1} = Pointcloud_ENU_ith_lap;
    end
end
   
%% Save to .mat and ply file
N_datasets = length(breakedCell);
clear Pointcloud_ENU_array
for idx_dataset = 1:N_datasets
    Pointcloud_Cell = breakedCell{idx_dataset};
    if isempty(Pointcloud_Cell)
        continue;
    end
    Pointcloud_ENU_array = cell2mat(Pointcloud_Cell);
    file_name = input("Please enter file name: ", "s");
    file_path = fullfile('D:\MappingVanData\PointcloudMap\Output',sprintf('%s.mat', file_name));
    save(file_path, 'Pointcloud_ENU_array','-v7.3');
    XYZ = Pointcloud_ENU_array(:,1:3);
    I = Pointcloud_ENU_array(:,4);
    ply_file_path = fullfile('D:\MappingVanData\PointcloudMap\Output',sprintf('%s.ply', file_name));
    pcwrite(pointCloud(XYZ,'Intensity',I), ply_file_path, 'PLYFormat','binary');  
end
%%
%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

%% function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
% Clear out the variables
clear global flag* FLAG*
clear flag*
clear path

% Clear out any path directories under Utilities
path_dirs = regexp(path,'[;]','split');
utilities_dir = fullfile(pwd,filesep,'Utilities');
for ith_dir = 1:length(path_dirs)
    utility_flag = strfind(path_dirs{ith_dir},utilities_dir);
    if ~isempty(utility_flag)
        rmpath(path_dirs{ith_dir});
    end
end

% Delete the Utilities folder, to be extra clean!
if  exist(utilities_dir,'dir')
    [status,message,message_ID] = rmdir(utilities_dir,'s');
    if 0==status
        error('Unable remove directory: %s \nReason message: %s \nand message_ID: %s\n',utilities_dir, message,message_ID);
    end
end

end % Ends fcn_INTERNAL_clearUtilitiesFromPathAndFolders

%% fcn_INTERNAL_initializeUtilities
function  fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders)
% Reset all flags for installs to empty
clear global FLAG*

fprintf(1,'Installing utilities necessary for code ...\n');

% Dependencies and Setup of the Code
% This code depends on several other libraries of codes that contain
% commonly used functions. We check to see if these libraries are installed
% into our "Utilities" folder, and if not, we install them and then set a
% flag to not install them again.

% Set up libraries
for ith_library = 1:length(library_name)
    dependency_name = library_name{ith_library};
    dependency_subfolders = library_folders{ith_library};
    dependency_url = library_url{ith_library};

    fprintf(1,'\tAdding library: %s ...',dependency_name);
    fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url);
    clear dependency_name dependency_subfolders dependency_url
    fprintf(1,'Done.\n');
end

% Set dependencies for this project specifically
fcn_DebugTools_addSubdirectoriesToPath(pwd,this_project_folders);

disp('Done setting up libraries, adding each to MATLAB path, and adding current repo folders to path.');
end % Ends fcn_INTERNAL_initializeUtilities


function fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url, varargin)
%% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES - MATLAB package installer from URL
%
% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES installs code packages that are
% specified by a URL pointing to a zip file into a default local subfolder,
% "Utilities", under the root folder. It also adds either the package
% subfoder or any specified sub-subfolders to the MATLAB path.
%
% If the Utilities folder does not exist, it is created.
%
% If the specified code package folder and all subfolders already exist,
% the package is not installed. Otherwise, the folders are created as
% needed, and the package is installed.
%
% If one does not wish to put these codes in different directories, the
% function can be easily modified with strings specifying the
% desired install location.
%
% For path creation, if the "DebugTools" package is being installed, the
% code installs the package, then shifts temporarily into the package to
% complete the path definitions for MATLAB. If the DebugTools is not
% already installed, an error is thrown as these tools are needed for the
% path creation.
%
% Finally, the code sets a global flag to indicate that the folders are
% initialized so that, in this session, if the code is called again the
% folders will not be installed. This global flag can be overwritten by an
% optional flag input.
%
% FORMAT:
%
%      fcn_DebugTools_installDependencies(...
%           dependency_name, ...
%           dependency_subfolders, ...
%           dependency_url)
%
% INPUTS:
%
%      dependency_name: the name given to the subfolder in the Utilities
%      directory for the package install
%
%      dependency_subfolders: in addition to the package subfoder, a list
%      of any specified sub-subfolders to the MATLAB path. Leave blank to
%      add only the package subfolder to the path. See the example below.
%
%      dependency_url: the URL pointing to the code package.
%
%      (OPTIONAL INPUTS)
%      flag_force_creation: if any value other than zero, forces the
%      install to occur even if the global flag is set.
%
% OUTPUTS:
%
%      (none)
%
% DEPENDENCIES:
%
%      This code will automatically get dependent files from the internet,
%      but of course this requires an internet connection. If the
%      DebugTools are being installed, it does not require any other
%      functions. But for other packages, it uses the following from the
%      DebugTools library: fcn_DebugTools_addSubdirectoriesToPath
%
% EXAMPLES:
%
% % Define the name of subfolder to be created in "Utilities" subfolder
% dependency_name = 'DebugTools_v2023_01_18';
%
% % Define sub-subfolders that are in the code package that also need to be
% % added to the MATLAB path after install; the package install subfolder
% % is NOT added to path. OR: Leave empty ({}) to only add
% % the subfolder path without any sub-subfolder path additions.
% dependency_subfolders = {'Functions','Data'};
%
% % Define a universal resource locator (URL) pointing to the zip file to
% % install. For example, here is the zip file location to the Debugtools
% % package on GitHub:
% dependency_url = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/blob/main/Releases/DebugTools_v2023_01_18.zip?raw=true';
%
% % Call the function to do the install
% fcn_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url)
%
% This function was written on 2023_01_23 by S. Brennan
% Questions or comments? sbrennan@psu.edu

% Revision history:
% 2023_01_23:
% -- wrote the code originally
% 2023_04_20:
% -- improved error handling
% -- fixes nested installs automatically

% TO DO
% -- Add input argument checking

flag_do_debug = 0; % Flag to show the results for debugging
flag_do_plots = 0; % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end


%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_check_inputs
    % Are there the right number of inputs?
    narginchk(3,4);
end

%% Set the global variable - need this for input checking
% Create a variable name for our flag. Stylistically, global variables are
% usually all caps.
flag_varname = upper(cat(2,'flag_',dependency_name,'_Folders_Initialized'));

% Make the variable global
eval(sprintf('global %s',flag_varname));

if nargin==4
    if varargin{1}
        eval(sprintf('clear global %s',flag_varname));
    end
end

%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if ~exist(flag_varname,'var') || isempty(eval(flag_varname))
    % Save the root directory, so we can get back to it after some of the
    % operations below. We use the Print Working Directory command (pwd) to
    % do this. Note: this command is from Unix/Linux world, but is so
    % useful that MATLAB made their own!
    root_directory_name = pwd;

    % Does the directory "Utilities" exist?
    utilities_folder_name = fullfile(root_directory_name,'Utilities');
    if ~exist(utilities_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(root_directory_name,'Utilities');

        % Did it work?
        if ~success_flag
            error('Unable to make the Utilities directory. Reason: %s with message ID: %s\n',error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The Utilities directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',error_message, message_ID);
        end

    end

    % Does the directory for the dependency folder exist?
    dependency_folder_name = fullfile(root_directory_name,'Utilities',dependency_name);
    if ~exist(dependency_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(utilities_folder_name,dependency_name);

        % Did it work?
        if ~success_flag
            error('Unable to make the dependency directory: %s. Reason: %s with message ID: %s\n',dependency_name, error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The %s directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',dependency_name, error_message, message_ID);
        end

    end

    % Do the subfolders exist?
    flag_allFoldersThere = 1;
    if isempty(dependency_subfolders{1})
        flag_allFoldersThere = 0;
    else
        for ith_folder = 1:length(dependency_subfolders)
            subfolder_name = dependency_subfolders{ith_folder};

            % Create the entire path
            subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);

            % Check if the folder and file exists that is typically created when
            % unzipping.
            if ~exist(subfunction_folder,'dir')
                flag_allFoldersThere = 0;
            end
        end
    end

    % Do we need to unzip the files?
    if flag_allFoldersThere==0
        % Files do not exist yet - try unzipping them.
        save_file_name = tempname(root_directory_name);
        zip_file_name = websave(save_file_name,dependency_url);
        % CANT GET THIS TO WORK --> unzip(zip_file_url, debugTools_folder_name);

        % Is the file there?
        if ~exist(zip_file_name,'file')
            error(['The zip file: %s for dependency: %s did not download correctly.\n' ...
                'This is usually because permissions are restricted on ' ...
                'the current directory. Check the code install ' ...
                '(see README.md) and try again.\n'],zip_file_name, dependency_name);
        end

        % Try unzipping
        unzip(zip_file_name, dependency_folder_name);

        % Did this work? If so, directory should not be empty
        directory_contents = dir(dependency_folder_name);
        if isempty(directory_contents)
            error(['The necessary dependency: %s has an error in install ' ...
                'where the zip file downloaded correctly, ' ...
                'but the unzip operation did not put any content ' ...
                'into the correct folder. ' ...
                'This suggests a bad zip file or permissions error ' ...
                'on the local computer.\n'],dependency_name);
        end

        % Check if is a nested install (for example, installing a folder
        % "Toolsets" under a folder called "Toolsets"). This can be found
        % if there's a folder whose name contains the dependency_name
        flag_is_nested_install = 0;
        for ith_entry = 1:length(directory_contents)
            if contains(directory_contents(ith_entry).name,dependency_name)
                if directory_contents(ith_entry).isdir
                    flag_is_nested_install = 1;
                    install_directory_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name);
                    install_files_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name,'*.*');
                    install_location_to = fullfile(directory_contents(ith_entry).folder);
                end
            end
        end

        if flag_is_nested_install
            [status,message,message_ID] = movefile(install_files_from,install_location_to);
            if 0==status
                error(['Unable to move files from directory: %s\n ' ...
                    'To: %s \n' ...
                    'Reason message: %s\n' ...
                    'And message_ID: %s\n'],install_files_from,install_location_to, message,message_ID);
            end
            [status,message,message_ID] = rmdir(install_directory_from);
            if 0==status
                error(['Unable remove directory: %s \n' ...
                    'Reason message: %s \n' ...
                    'And message_ID: %s\n'],install_directory_from,message,message_ID);
            end
        end

        % Make sure the subfolders were created
        flag_allFoldersThere = 1;
        if ~isempty(dependency_subfolders{1})
            for ith_folder = 1:length(dependency_subfolders)
                subfolder_name = dependency_subfolders{ith_folder};

                % Create the entire path
                subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);

                % Check if the folder and file exists that is typically created when
                % unzipping.
                if ~exist(subfunction_folder,'dir')
                    flag_allFoldersThere = 0;
                end
            end
        end
        % If any are not there, then throw an error
        if flag_allFoldersThere==0
            error(['The necessary dependency: %s has an error in install, ' ...
                'or error performing an unzip operation. The subfolders ' ...
                'requested by the code were not found after the unzip ' ...
                'operation. This suggests a bad zip file, or a permissions ' ...
                'error on the local computer, or that folders are ' ...
                'specified that are not present on the remote code ' ...
                'repository.\n'],dependency_name);
        else
            % Clean up the zip file
            delete(zip_file_name);
        end

    end


    % For path creation, if the "DebugTools" package is being installed, the
    % code installs the package, then shifts temporarily into the package to
    % complete the path definitions for MATLAB. If the DebugTools is not
    % already installed, an error is thrown as these tools are needed for the
    % path creation.
    %
    % In other words: DebugTools is a special case because folders not
    % added yet, and we use DebugTools for adding the other directories
    if strcmp(dependency_name(1:10),'DebugTools')
        debugTools_function_folder = fullfile(root_directory_name, 'Utilities', dependency_name,'Functions');

        % Move into the folder, run the function, and move back
        cd(debugTools_function_folder);
        fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        cd(root_directory_name);
    else
        try
            fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        catch
            error(['Package installer requires DebugTools package to be ' ...
                'installed first. Please install that before ' ...
                'installing this package']);
        end
    end


    % Finally, the code sets a global flag to indicate that the folders are
    % initialized.  Check this using a command "exist", which takes a
    % character string (the name inside the '' marks, and a type string -
    % in this case 'var') and checks if a variable ('var') exists in matlab
    % that has the same name as the string. The ~ in front of exist says to
    % do the opposite. So the following command basically means: if the
    % variable named 'flag_CodeX_Folders_Initialized' does NOT exist in the
    % workspace, run the code in the if statement. If we look at the bottom
    % of the if statement, we fill in that variable. That way, the next
    % time the code is run - assuming the if statement ran to the end -
    % this section of code will NOT be run twice.

    eval(sprintf('%s = 1;',flag_varname));
end

%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_plots

    % Nothing to do!



end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends function fcn_DebugTools_installDependencies
