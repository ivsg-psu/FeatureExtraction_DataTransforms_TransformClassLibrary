function lockedDataStruct = fcn_DataClean_RemoveUnlockedData(dataStructure)

% fcn_DataClean_RemoveUnlockedData removes unlocked GPS data from dataStructure
%
% FORMAT:
%
% lockedDataStruct = fcn_DataClean_RemoveUnlockedData(dataStructure,method)
%
% INPUTS:
%
%      dataStructure: a structure array containing sensor fileds
%
%
% OUTPUTS:
%
%      lockedDataStruct: a structure array containing locked GPS data
%
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%      fcn_geometry_plotCircle
%
% EXAMPLES:
%      
% See the script: 
% for a full test suite.
%
% This function was written on 2023_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2023_10_20 - wrote the code
% 2024_01_28 - added more comments, particularly to explain inputs more
% clearly

%% Debugging and Input checks
flag_do_debug = 0;
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
flag_check_inputs = 1; % Flag to perform input checking

if flag_check_inputs == 1
    if ~isstruct(dataStructure)
        error('The input of the function should be a structure array')
    end
end

%% Solve for the angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create a new structure array has the same fields with GPS_rawdata_struct
lockedDataStruct = dataStructure;
fields = fieldnames(dataStructure);

gps_indx = cellfun(@(x) contains(x, "GPS"), fields);
gps_fields = fields(gps_indx);
N_gps_fields = length(gps_fields);
for ith_gps_field = 1:N_gps_fields
    gps_name = gps_fields{ith_gps_field};
    GPS_rawdata_struct = dataStructure.(gps_name);
    GPS_locked_struct = GPS_rawdata_struct;
    % Grab the lock status from the rawdata
    LockStatus = GPS_rawdata_struct.DGPS_mode;
    AgeOfDiff = GPS_rawdata_struct.AgeOfDiff;
    % Find the locked data indexs
    idxs_locked = find((LockStatus>5)&(AgeOfDiff < 20));

    topic_fields = fieldnames(GPS_rawdata_struct);
    N_topics = length(topic_fields);
    for i_field = 1:N_topics
        current_field_array = GPS_rawdata_struct.(topic_fields{i_field});
        % Fill the fields with arrays
        if length(current_field_array)>1
            GPS_locked_struct.(topic_fields{i_field}) = current_field_array(idxs_locked,:);
        end
    end
    if ~isempty(idxs_locked)
        GPS_locked_struct.centiSeconds = GPS_rawdata_struct.centiSeconds;
        GPS_locked_struct.Npoints = length(GPS_rawdata_struct.Latitude);
    else
        GPS_locked_struct = [];
    end
    lockedDataStruct.(gps_name) = GPS_locked_struct;
end


end