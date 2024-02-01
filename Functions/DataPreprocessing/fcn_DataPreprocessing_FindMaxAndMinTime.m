function [time_range] = fcn_DataPreprocessing_FindMaxAndMinTime(rawDataLocked)
    fields = fieldnames(rawDataLocked);
    time_start = -Inf;
    time_end = Inf;
    for idx_field = 1:length(fields)
        current_field_struct = rawDataLocked.(fields{idx_field});
        if contains(fields{idx_field},"GPS")
            current_field_struct_time = current_field_struct.ROS_Time*(10^-9);
        else
            current_field_struct_time = current_field_struct.ROS_Time;
        end
        time_start = max([min(current_field_struct_time),time_start]);
        time_end = min([max(current_field_struct_time),time_end]);
        

    end
    time_range = [time_start,time_end];

    
end