function [matchedSensorNames] = fcn_DataClean_findMatchingSensors(dataStructure, sensor_identifier_string, varargin)

% fcn_DataClean_findMatchingSensors
% Finds which sensors have names that match a given query
%
% FORMAT:
%
%      matchedSensorNames = fcn_DataClean_findMatchingSensors(dataStructure, sensor_identifier_string, (fid))
%
% INPUTS:
%
%      dataStructure: a data structure to be analyzed 
%
%      sensor_identifier_string: a string that is used to select only some
%      sensor fields, matched using lower-case comparisons. For example, if
%      sensor_identifier_string = 'GPS', then all sensors whose names, when
%      coverted to lower case, contain 'gps' will be queried. If there are
%      only 3 sensors with 'gps' in their name, then the resulting
%      dataArray and sensorNames will only have 3 entries.
% 
%      (OPTIONAL INPUTS)
%
%      fid: a file ID to print results of analysis. If not entered, no
%      printing is done. Set fid to 1 to print to the console.
%
% OUTPUTS:
%
%      matchedSensorNames: a structure containing an array of each sensor's name
%      that contains sensor_identifier_string
% 
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%
% EXAMPLES:
%
%     See the script: script_test_fcn_DataClean_findMatchingSensors
%     for a full test suite.
%
% This function was written on 2023_07_04 by S. Brennan
% Questions or comments? sbrennan@psu.edu 

% Revision history:
%     
% 2023_07_04: sbrennan@psu.edu
% -- wrote the code originally 
% 2024_09_26: sbrennan@psu.edu
% -- updated to comments
% -- added debug flag area
% -- fixed fid printing error

%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
flag_max_speed = 0;
if (nargin==3 && isequal(varargin{end},-1))
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
        narginchk(2,3);

    end
end

% Does the user want to specify the fid?
fid = 0; % Default is 0 (not printing)
if (0 == flag_max_speed) && (3 <= nargin)
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

flag_do_plots = 0; % Shut off plotting

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
sensor_names = fieldnames(dataStructure); % Grab all the fields that are in dataStructure structure

% Grab the list of matches
matched_indicies = contains(lower(sensor_names),lower(sensor_identifier_string));
Nsensors = sum(matched_indicies);

% If there were no matches, just exit now
if Nsensors==0
    matchedSensorNames = '';
    return
end

% Keep only the ones that match
matchedSensorNames = sensor_names(matched_indicies);



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
    fprintf(fid,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
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

