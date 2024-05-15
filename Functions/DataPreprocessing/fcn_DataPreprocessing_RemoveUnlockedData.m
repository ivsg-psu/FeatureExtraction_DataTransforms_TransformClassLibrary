function GPS_Locked_data_struct = fcn_DataPreprocessing_RemoveUnlockedData(GPS_rawdata_struct)

% fcn_DataPreprocessing_RemoveUnlockedData removes unlocked GPS data from
% the single GPS_data_struct
%
% FORMAT:
%
% GPS_Locked = fcn_DataPreprocessing_RemoveUnlockedData(GPS_rawdata_struct)
%
% INPUTS:
%
%      GPS_rawdata_struct: a structure array containing raw GPS data
%
%
% OUTPUTS:
%
%      GPS_Locked_data_struct: a structure array containing locked GPS data
%
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%      fcn_geometry_plotCircle
%
% EXAMPLES:
%      
%      % BASIC example
%      points = [0 0; 1 4; 0.5 -1];
%      [centers,radii] = fcn_geometry_circleCenterFrom3Points(points,1)
% 
% See the script: script_test_fcn_Transform_CalculateAngleBetweenVectors
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
    if ~isstruct(GPS_rawdata_struct)
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
GPS_Locked_data_struct = GPS_rawdata_struct;
% Grab the lock status from the rawdata
LockStatus = GPS_rawdata_struct.DGPS_mode;
% Find the locked data indexs
idxs_locked = find(LockStatus>5);
fns = fieldnames(GPS_Locked_data_struct);
N_fields = length(fns);
for i_field = 1:N_fields
    current_field_array = GPS_rawdata_struct.(fns{i_field});
    % Fill the fields with arrays
    if ~isscalar(current_field_array)
        GPS_Locked_data_struct.(fns{i_field}) = current_field_array(idxs_locked,:);
    end
end
if ~isempty(idxs_locked)
    GPS_Locked_data_struct.centiSeconds = GPS_rawdata_struct.centiSeconds;
    GPS_Locked_data_struct.Npoints = length(GPS_Locked_data_struct.Latitude);
else
    GPS_Locked_data_struct.centiSeconds = [];
    GPS_Locked_data_struct.Npoints = [];
end
end