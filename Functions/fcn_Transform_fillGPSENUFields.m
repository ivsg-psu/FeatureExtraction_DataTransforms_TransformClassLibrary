function convertedDataStructure = fcn_Transform_fillGPSENUFields(dataStructure, ref_basestation)
% fcn_Transform_fillGPSENUFields
%
% Converts all GPS fields within the input data structure from LLA 
% (latitude, longitude, altitude) to ENU (East, North, Up) coordinates and
% fill corresponding fields
%
% INPUTS:
%   dataStructure      - A structure containing various sensor fields including GPS
%   ref_basestation    - A 1x3 vector [lat, lon, alt] used as the ENU reference origin
%
% OUTPUT:
%   convertedDataStructure - The updated structure where GPS fields have new fields:
%                             xEast, yNorth, zUp (in meters, ENU frame)
%
% NOTE:
%   This function looks for fields containing 'GPS' in their names.
%   The conversion uses the 'lla2enu' function with the WGS84 ellipsoid model.

% This function was written on 2025_05_17 by X. Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
%     
% 2025_05_17: xfc5113@psu.edu
% -- wrote the code originally 
% 2025_11_23: xfc5113@psu.edu
% -- renamed the function to fcn_Transform_fillGPSENUFields

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

% Extract all field names from the structure
fields = fieldnames(dataStructure);
N_fields = length(fields);
convertedDataStructure = dataStructure;

for idx_field = 1:N_fields
    field_name = fields{idx_field};
    
    % Only process fields related to GPS
    if contains(field_name, 'GPS')
        GPS_struct = dataStructure.(field_name);
        converted_GPS_struct = GPS_struct;

        % Extract LLA components
        GPS_Latitude  = GPS_struct.Latitude;
        GPS_Longitude = GPS_struct.Longitude;
        GPS_Altitude  = GPS_struct.Altitude;
        GeoSep = GPS_struct.GeoSep;
        GPS_Altitude_normalized = GPS_Altitude - GeoSep;
        GPS_LLA       = [GPS_Latitude, GPS_Longitude, GPS_Altitude_normalized];  % Nx3
        
        % Convert to ENU coordinates
        GPS_ENU = lla2enu(GPS_LLA, ref_basestation, 'ellipsoid');

        % Assign ENU values to new fields
        converted_GPS_struct.xEast  = GPS_ENU(:,1);
        converted_GPS_struct.yNorth = GPS_ENU(:,2);
        converted_GPS_struct.zUp    = GPS_ENU(:,3);

        % Optional: store ENU arrays based on sensor position keyword
        % if contains(field_name, 'Front')
        %     GPS_Front_ENU_array = GPS_ENU; %#ok<NASGU> 
        % elseif contains(field_name, 'Left')
        %     GPS_LeftRear_ENU_array = GPS_ENU; %#ok<NASGU> 
        % elseif contains(field_name, 'Right')
        %     GPS_RightRear_ENU_array = GPS_ENU; %#ok<NASGU> 
        % end

        % Update the structure with converted GPS
        convertedDataStructure.(field_name) = converted_GPS_struct;
    end
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


end