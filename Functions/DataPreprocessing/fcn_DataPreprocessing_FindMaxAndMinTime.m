function [time_range] = fcn_DataPreprocessing_FindMaxAndMinTime(rawDataLocked)

% fcn_DataPreprocessing_FindMaxAndMinTime finds the start and end time for
% each sensor
%
% FORMAT:
%
% time_range = fcn_DataPreprocessing_FindMaxAndMinTime(rawDataLocked)
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
flag_do_debug = 1;
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
    if ~isstruct(rawDataLocked)
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


    fields = fieldnames(rawDataLocked);
    time_start = -Inf;
    time_end = Inf;
    for idx_field = 1:length(fields)
        current_field_struct = rawDataLocked.(fields{idx_field});
        if ~isempty(current_field_struct)
            if contains(fields{idx_field},"GPS")
                current_field_struct_time = current_field_struct.ROS_Time*(10^-9);
            else
                current_field_struct_time = current_field_struct.ROS_Time;
            end
        end
        time_start = max([min(current_field_struct_time),time_start]);
        time_end = min([max(current_field_struct_time),time_end]);
        

    end
    time_range = [time_start,time_end];

    
end