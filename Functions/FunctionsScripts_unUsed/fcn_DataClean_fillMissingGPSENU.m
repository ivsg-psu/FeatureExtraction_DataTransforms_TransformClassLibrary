function filled_dataStructure = fcn_DataClean_fillMissingGPSENU(dataStructure, varargin)
%% NOTE: This function is a temp version that used to fill missing in GPS units
% fcn_DataClean_fillMissingGPSENU fills missing ENU data for all GPS units
%
% FORMAT:
%
% filled_dataStructure = fcn_DataClean_fillMissingGPSENU(dataStructure, (fid))
%
% INPUTS:
%
%       dataStructure: a data structure array contains time cleaned and ENU
%       calculated data
%
%       (OPTIONAL INPUTS)
%
%       fid: a file ID to print results of analysis. If not entered, the
%       console (FID = 1) is used.
%
%
% OUTPUTS:
%
%      filled_dataStructure: a data structure array contains ENU filled data
%
% DEPENDENCIES:
%
%
% EXAMPLES:
%
%
% This function was written on 2024_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2024_10_20 - wrote the code


flag_do_debug = 0;
flag_check_inputs = 1;
if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    % fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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
    narginchk(1,2);
end



% Does user want to specify fid?
fid = 0;
if 2 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        fid = temp;
    end
end


if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end



%% Grab GPS units
[~, sensor_names_GPS_Time] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, 'GPS_Time','GPS');

%% Convert LLA to ENU for all GPS units
filled_dataStructure = dataStructure;
for idx_gps_unit = 1:length(sensor_names_GPS_Time)
    GPSUnitName = sensor_names_GPS_Time{idx_gps_unit};
    GPSdataStructure = dataStructure.(GPSUnitName);
    filled_GPSdataStructure = GPSdataStructure;
    GPS_xEast = GPSdataStructure.xEast;
    GPS_yNorth = GPSdataStructure.yNorth;
    GPS_zUp = GPSdataStructure.zUp;
    GPS_DGPS_mode = GPSdataStructure.DGPS_mode;
    GPS_xEast_filled = fillmissing(GPS_xEast,'linear');
    GPS_yNorth_filled = fillmissing(GPS_yNorth,'linear');
    GPS_zUp_filled = fillmissing(GPS_zUp,'linear');
    GPS_DGPS_mode_filled = fillmissing(GPS_DGPS_mode,'next');

    filled_GPSdataStructure.xEast = GPS_xEast_filled;
    filled_GPSdataStructure.yNorth = GPS_yNorth_filled;
    filled_GPSdataStructure.zUp = GPS_zUp_filled;
    filled_GPSdataStructure.DGPS_mode = GPS_DGPS_mode_filled;
    
    filled_dataStructure.(GPSUnitName) = filled_GPSdataStructure;
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

if flag_do_debug
    if fid~=0
        fprintf(fid,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
    end
end

end