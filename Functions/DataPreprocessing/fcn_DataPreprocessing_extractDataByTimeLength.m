function extracted_data_struct = fcn_DataPreprocessing_extractDataByTimeLength(data_struct,time_length, direction)

% fcn_DataPreprocessing_extractDataByTimeLength extracts the data given by
% the time length, if the total time length is shorted than the specified
% time length, extract all data.
%
% FORMAT:
%
% extracted_data_struct = fcn_DataPreprocessing_extractDataByTimeLength(data_struct,time_length, direction)
%
% INPUTS:
%
%      data_struct: a structure array containing data of one sensor
%
%      time_length: a scalar indicates the length of time that will be
%      extracted, unit: second [s]
%
%      direction: a scalar indicates the direction of extraction, 1 from
%      the first element, -1 from the last element
%
% OUTPUTS:
%
%      extracted_data_struct: a structure array containing extracted data
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
    if ~isstruct(data_struct)
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
extracted_data_struct = data_struct;
% Grab number of points in the data struct
Npoints = data_struct.Npoints;
centiSeconds = data_struct.centiSeconds;
N_extracted_points = time_length*centiSeconds;
% If the required number of data is smaller than the total number of data
% points, extract the required data in the data structure
if N_extracted_points<=Npoints
    fns = fieldnames(extracted_data_struct);
    N_fields = length(fns);
    if direction == 1
        idxs_extraction = 1:N_extracted_points;

    elseif direction == -1
        idxs_extraction = (Npoints-N_extracted_points+1):Npoints;
    end
    for i_field = 1:N_fields
        current_field_array = data_struct.(fns{i_field});
    % Fill the fields with arrays
    
        if ~isscalar(current_field_array)&~isempty(current_field_array)
            extracted_data_struct.(fns{i_field}) = current_field_array(idxs_extraction,:);
        else
            if any(contains(fns{i_field},'Npoints'))
                extracted_data_struct.Npoints = N_extracted_points;
            end
        end

    end
end

end