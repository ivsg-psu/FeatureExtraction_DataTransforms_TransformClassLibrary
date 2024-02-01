function calibrate_matrix = fcn_Transform_createCalibrateMatrix(yaw_offset, pitch_offset, roll_offset)


R_z = rotz(rad2deg(yaw_offset));
R_y = roty(rad2deg(pitch_offset));
R_x = rotz(rad2deg(roll_offset));

calibrate_matrix = R_z*R_y*R_x;