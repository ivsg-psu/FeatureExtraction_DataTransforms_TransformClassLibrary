function roll_offset_array = fcn_Transform_calculateRollOffset(V_y_virtual_1,V_y_virtual_2)
% Note: This function is very abstract, containing a virtual frame
% that do not exist, will add figures to explain the function
% Vehicle Coordinate Frame： X-Y-Z
% V_x_vehicle: X-axis of Vehicle Coordinate Frame
% V_y_vehicle: Y-axis of Vehicle Coordinate Frame
% V_z_vehicle: Z-axis of Vehicle Coordinate Frame

% Virtual Coordinate Frame： X'-Y'-Z'
% V_x_virtual = V_x_vehicle;
% V_y_virtual: Y-axis of a virtual frame used to calculate offset
% V_z_virtual: Z-axis of a virtual frame used to calculate offset
%% Step 1 - Preprocess the GPS data, all unlocked data will be removed and time will be synchronized (Temp version, will update later)
%% Roll

angle_diff_mag = fcn_Transform_CalculateAngleBetweenVectors(V_y_virtual_1,V_y_virtual_2);
roll_offset_mag = angle_diff_mag/2;
V_roll_sign = cross(V_y_virtual_1,V_y_virtual_2);
roll_direction = sign(V_roll_sign(:,1));
roll_offset_array = roll_direction.*roll_offset_mag;

end
%% Normalize vector to unit vector
function V_unit = normalizeVector(V)
    V_mag = vecnorm(V,2,2);
    V_unit = V./V_mag;
end
