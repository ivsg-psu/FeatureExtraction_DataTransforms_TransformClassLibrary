function vehiclePose_DuplicatesUpdated = fcn_Transform_generateVehiclePoseDuplicates(vehiclePose, SensorData)
%fcn_Transform_generateVehiclePoseDuplicates
%
% This function takes the original vehicle pose data and sensor data. The
% number of elements in the original data is not equal to the number of
% elements in the sensor data. This function updates the vehicle pose data
% by duplicating the elements in the original vehicle pose data to make the
% size of the vehiclePose data (,1) equal to the size of the sensor data.
% This function is typically used with LiDAR and camera data
%
% METHOD
%
% 1) The length of the both input matrices (vehiclePose and SensorData) is 
% computed.
%
% 2) Then the duplicates needed for each element in vehicle pose is
% calculated by dividing the no. of elements in sensor data by no. of
% elements in vehicle pose. Ceil of this division is calculated and stored
% in duplicates_required variable.
%
% 3) Based on "duplicates_required", the duplicates are stored in
% "vehiclePose_DuplicatesUpdated" matrix. 
%
% FORMAT
%
% vehiclePose_DuplicatesUpdated = fcn_Transform_generateVehiclePoseDuplicates(vehiclePose, SensorData)
%
% INPUTS
% 
% vehiclePose: The original vehicle pose data which contains less number of
% elements compared to sensor data
%
% SensorData: This sensor data is typically LiDAR or camera data
%
% OUTPUT
% 
% vehiclePose_DuplicatesUpdated: The vehicle pose which contains the
% duplicates and the length of this matrix matches with sensor data
%
% DEPENDENCIES:
% 
% None
% 
% EXAMPLES:
% 
% See the script: script_test_fcn_Transform_generateVehiclePoseDuplicates
% for a full test suite.

% Revision History
% 2023_10_12: Aneesh Batchu
% -- wrote the code originally.

flag_do_debug = 0;  % Flag to show the results for debugging
flag_do_plots = 0;  % % Flag to plot the final results
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
    narginchk(2,2);

end

if flag_do_debug
        flag_do_plots = 1;
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
%% 

Nelements_SensorData = size(SensorData,1);
Nelements_vehiclePose = size(vehiclePose, 1);

% The duplicates needed for each vehiclePose element
duplicates_required = ceil(Nelements_SensorData/Nelements_vehiclePose);

% Elements required in updated vehicle Pose
vehiclePose_DuplicatesUpdated = ones(Nelements_vehiclePose*duplicates_required, 6);

% First element of the vehicle pose is duplicated
vehiclePose_DuplicatesUpdated(1:duplicates_required,:) = vehiclePose(1,:).*vehiclePose_DuplicatesUpdated(1:duplicates_required,:);

% Remaining elements (other than 1) are duplicated
for i = 2:Nelements_vehiclePose
    vehiclePose_DuplicatesUpdated((i-1)*duplicates_required + 1:i*duplicates_required,:) = vehiclePose(i,:).*vehiclePose_DuplicatesUpdated((i-1)*duplicates_required + 1:i*duplicates_required,:);
end

% Final updated vehiclePose with duplicates
vehiclePose_DuplicatesUpdated = vehiclePose_DuplicatesUpdated(1:Nelements_SensorData,:);

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
    % No plots to show
end
end