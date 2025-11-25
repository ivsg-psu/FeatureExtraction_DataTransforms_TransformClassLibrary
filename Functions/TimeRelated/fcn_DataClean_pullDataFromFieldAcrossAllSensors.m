function [dataArray, sensorNames] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, field_string, varargin)
% fcn_DataClean_pullDataFromFieldAcrossAllSensors
% Pulls a given field's data from all sensors. If the field does not exist,
% it returns an empty array for that field
%
% FORMAT:
%
%      dataArray = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, field_string, (sensor_identifier_string), (entry_location), (fid), (fig_num))
%
% INPUTS:
%
%      dataStructure: a data structure to be analyzed 
%
%      field_string: the field whose data is to be collected
%
%      (OPTIONAL INPUTS)
%
%      sensor_identifier_string: a string that is used to select only some
%      sensor fields, matched using lower-case comparisons. For example, if
%      sensor_identifier_string = 'GPS', then all sensors whose names, when
%      coverted to lower case, contain 'gps' will be queried. If there are
%      only 3 sensors with 'gps' in their name, then the resulting
%      dataArray and sensorNames will only have 3 entries.
% 
%      entry_location: a string specifying which element to keep, if the
%      data is an array:
%      
%            'first_row' - saves the first element of the first column, i.e. (1,1)
%            'last_row'  - saves the last element of the first column, i.e. (end,1)
%            'all'   - (default) saves the entire array
%
%      fid: a file ID to print results of analysis. If not entered, no
%      printing is done. Set fid to 1 to print to the console.
%
%      fig_num: a figure number to plot results. If set to -1, skips any
%      input checking or debugging, no figures will be generated, and sets
%      up code to maximize speed.
%
% OUTPUTS:
%
%      dataArray: a data structure in cell array form containing the data
%      from every sensor
%
%      sensorNames: a structure containing an array of each sensor's name
% 
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
%     See the script: script_test_fcn_DataClean_pullDataFromFieldAcrossAllSensors
%     for a full test suite.
%
% This function was written on 2023_06_29 by S. Brennan
% Questions or comments? sbrennan@psu.edu 

% Revision history:
%     
% 2023_06_29: Sean Brennan, sbrennan@psu.edu
% -- wrote the code originally 
% 2023_07_04: Sean Brennan, sbrennan@psu.edu
% -- added narginchk
% 2024_09_10: Sean Brennan, sbrennan@psu.edu
% -- fixed bug where it breaks if query 1st or last entry in empty field
% -- added debug modes
% -- added fig_num input for speed

% TO DO

%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
flag_max_speed = 0;
if (nargin==6 && isequal(varargin{end},-1))
    flag_do_debug = 0; % % % % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; % % % % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_DATACLEAN_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_DATACLEAN_FLAG_CHECK_INPUTS");
    MATLABFLAG_DATACLEAN_FLAG_DO_DEBUG = getenv("MATLABFLAG_DATACLEAN_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_DATACLEAN_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_DATACLEAN_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_DATACLEAN_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_DATACLEAN_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

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

if 0 == flag_max_speed
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(2,6);

    end
end

% Does the user want to specify the sensor_identifier_string
sensor_identifier_string = '';
if 3 <= nargin
    temp = varargin{1};
    if ~isempty(temp)
        sensor_identifier_string = temp;
    end
end

% Does the user want to specify the entry_location?
entry_location = 'all';
if 4 <= nargin
    temp = varargin{2};
    if ~isempty(temp)
        entry_location = temp;
    end
end


% Does the user want to specify the fid?
fid = 0; % Default is 0 (not printing)
if (0==flag_max_speed) && (5 == nargin)
    temp = varargin{end};
    if ~isempty(temp)
        % Check that the FID works
        try
            temp_msg = ferror(temp); %#ok<NASGU>
            % Set the fid value, if the above ferror didn't fail
            fid = temp;
        catch ME
            warning('on','backtrace');
            warning('User-specified FID does not correspond to a file. Unable to continue.');
            throwAsCaller(ME);
        end
    end
end

% Does user want to specify fig_num?
flag_do_plots = 0;
if (0==flag_max_speed) &&  (5<=nargin)
    temp = varargin{end};
    if ~isempty(temp)
        fig_num = temp; %#ok<NASGU>
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

% Produce a list of all the sensors (each is a field in the structure)
sensorNames = fcn_DataClean_findMatchingSensors(dataStructure, sensor_identifier_string);
Nsensors = length(sensorNames);

if Nsensors==0
    dataArray{1} = [];
    sensorNames = '';
    return
end

% Initialize the data array
dataArray{Nsensors}        = [];

if 0~=fid
    fprintf(fid,'\nPulling data from field %s across all sensors:\n',field_string);
end

% Loop through the fields, searching for ones that have sensor_identifier_string in their name
for ith_sensor = 1:Nsensors

    % Grab the sensor subfield name
    sensor_name = sensorNames{ith_sensor};
    sensor_data = dataStructure.(sensor_name);

    % Tell what we are doing
    if 0~=fid
        fprintf(fid,'\t Loading data from sensor %d of %d: %s\n',ith_sensor,length(sensorNames),sensor_name);
    end

    % Does the field exist?
    if isfield(sensor_data, field_string)
        dataArray{ith_sensor} = sensor_data.(field_string);
    else
        dataArray{ith_sensor} = [];
    end
end

%            'first_row' - saves the first element of the first column, i.e. (1,1)
%            'last_row'  - saves the last element of the first column, i.e. (end,1)
%            'all'   - (default) saves the entire array

switch entry_location
    case 'all'
        % Do nothing, return
    case 'first_row' % saves the first element of the first column, i.e. (1,1)
        for ith_sensor = 1:Nsensors
            if ~isempty(dataArray{ith_sensor}) && length(dataArray{ith_sensor}(:,1))>=1
                dataArray{ith_sensor} = dataArray{ith_sensor}(1,1);
            end
        end
    case 'last_row' % saves the first element of the first column, i.e. (1,1)
        for ith_sensor = 1:Nsensors
            if ~isempty(dataArray{ith_sensor}) && length(dataArray{ith_sensor}(:,1))>=1
                dataArray{ith_sensor} = dataArray{ith_sensor}(end,1);
            end
        end
    otherwise
        error('unrecognized position requested for data retrieval');
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
    
    % Nothing to plot        
    
end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends main function




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

