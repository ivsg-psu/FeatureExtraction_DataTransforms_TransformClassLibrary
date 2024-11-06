function converted_dataStructure = fcn_Transform_convertLLA2ENU(dataStructure, varargin)

% fcn_Transform_convertLLA2ENU
% Convert all GPS LLA to ENU and save in the dataStructure
%
%
% FORMAT:
%
%      converted_dataStructure = fcn_Transform_convertLLA2ENU(dataStructure,(ref_baseStationLLA),(fid))
%
% INPUTS:
%
%      dataStructure: a data structure to be analyzed that includes the following
%      fields:
%
%      (OPTIONAL INPUTS)
%       
%      ref_baseStationLLA: a array contain the LLA coordinate of the
%      reference base station
%
%      fid: a file ID to print results of analysis. If not entered, the
%      console (FID = 1) is used.
% 
%      fig_num: a figure number to plot results. If set to -1, skips any
%      input checking or debugging, no figures will be generated, and sets
%      up code to maximize speed.
%
% OUTPUTS:
%
%      converted_dataStructure: a data structure to be analyzed that includes the following
%      fields:
% 
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
%     See the script: script_test_fcn_Transform_convertLLA2ENU
%     for a full test suite.
%
% This function was written on 2024_10_13 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
%     
% 2024_09_22: xfc5113@psu.edu
% -- wrote the code originally

%% Debugging and Input checks

flag_do_debug = 0; % % % % Flag to plot the results for debugging
flag_check_inputs = 1; % Flag to perform input checking


if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
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
    narginchk(1,4);
end


ref_baseStationLLA = [40.86368573 -77.83592832 344.189];
if 2 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        ref_baseStationLLA = temp;
    end
end

% Does the user want to specify the fid?
fid = 0;
if 3 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        % Check that the FID works
        % Set the fid value, if the above ferror didn't fail
        fid = temp;  
    end
end
% Does the user want to specify the fig_num?
fig_num = -1;
flag_do_plots = 0;
if 4 <= nargin
    temp = varargin{3};
    if ~isempty(temp)
        fig_num = temp;
        flag_do_plots = 1;       
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

%% Grab GPS units
[~, sensor_names_GPS_Time] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, 'GPS_Time','GPS');

%% Convert LLA to ENU for all GPS units
converted_dataStructure = dataStructure;
for idx_gps_unit = 1:length(sensor_names_GPS_Time)
    GPSUnitName = sensor_names_GPS_Time{idx_gps_unit};
    GPSdataStructure = dataStructure.(GPSUnitName);
    GPSLLA_array = [GPSdataStructure.Latitude, GPSdataStructure.Longitude, GPSdataStructure.Altitude];
    GPSENU_array = lla2enu(GPSLLA_array,ref_baseStationLLA,'ellipsoid');
    % Fill the ENU to the GPS dataStructure
    GPSdataStructure.xEast = GPSENU_array(:,1);
    GPSdataStructure.yNorth = GPSENU_array(:,2);
    GPSdataStructure.zUp = GPSENU_array(:,3);
    converted_dataStructure.(GPSUnitName) = GPSdataStructure;
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

    % check whether the figure already has data
    figure(fig_num);
    plot_colors = ['r','g','b'];
    for idx_gps_unit = 1:length(sensor_names_GPS_Time)

        GPSdataStructure = converted_dataStructure{idx_gps_unit};
        GPSENU_array = [GPSdataStructure.xEast,GPSdataStructure.yNorth,GPSdataStructure.zUp];

        scatter3(GPSENU_array(:,1), GPSENU_array(:,2),GPSENU_array(:,3),30,plot_colors(idx_gps_unit),'filled');
        hold on
        
    end 
    legend(sensor_names_GPS_Time);
    xlabel('x-East [m]');
    ylabel('y-North [m]')
    zlabel('z-Up [m]')
    % title(sprintf('%s',sensor_names_GPS_Time{idx_gps_unit}),'interpreter','none','FontSize',12)
end

if flag_do_debug
    if fid > 0
        fprintf(fid,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
    end
end
