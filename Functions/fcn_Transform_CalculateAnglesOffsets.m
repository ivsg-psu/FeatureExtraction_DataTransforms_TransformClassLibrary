function [yaw_offset_array, pitch_offset_array, roll_offset_array] = fcn_Transform_CalculateAnglesOffsets(rawdata_1,rawdata_2,process_range)


[yaw_offset_array_1, pitch_offset_array_1,V_virtual_frame_1] = fcn_Transform_calculateYawAndPitchOffset(rawdata_1,process_range);
[yaw_offset_array_2, pitch_offset_array_2,V_virtual_frame_2] = fcn_Transform_calculateYawAndPitchOffset(rawdata_2,process_range);
yaw_offset_array = [yaw_offset_array_1;yaw_offset_array_2];
pitch_offset_array = [pitch_offset_array_1;pitch_offset_array_2];
V_y_virtual_1 = V_virtual_frame_1.Y;
V_y_virtual_2 = V_virtual_frame_2.Y;
%% Roll

roll_offset_array = fcn_Transform_calculateRollOffset(V_y_virtual_1,V_y_virtual_2);