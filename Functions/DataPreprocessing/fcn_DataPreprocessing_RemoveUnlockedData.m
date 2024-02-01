function GPS_Locked = fcn_DataPreprocessing_RemoveUnlockedData(data_struct)
    GPS_Locked = data_struct;
    LockStatus = GPS_Locked.DGPS_mode;
    start_idx = find(LockStatus>5, 1);
    last_idx = GPS_Locked.Npoints;
    valid_idxs = start_idx:last_idx;
    
    fns = fieldnames(data_struct);
    N_fields = length(fns);
    for i_field = 1:N_fields
        current_field_array = data_struct.(fns{i_field});
        if length(current_field_array) > 1
            GPS_Locked.(fns{i_field}) = current_field_array(valid_idxs,:);
        end
    end
    
    if ~isempty(valid_idxs)
        GPS_Locked.centiSeconds = data_struct.centiSeconds;
        GPS_Locked.Npoints = length(GPS_Locked.ROS_Time);
    else
        GPS_Locked.centiSeconds = [];
        GPS_Locked.Npoints = [];
    end
end