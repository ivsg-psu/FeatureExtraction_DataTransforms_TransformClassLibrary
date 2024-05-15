function valid_struct = fcn_DataPreprocessing_SelectValidData(data_struct,time_range,datatype)
    if isempty(data_struct)||isempty(time_range)
        valid_struct = [];
    else
        if datatype == "gps"
           time_array = data_struct.ROS_Time*10^-9;
           
        elseif datatype == "lidar"
           time_array = data_struct.ROS_Time;
    
        end
        valid_struct = data_struct;
        valid_idxs = (time_array>=min(time_range)) & (time_array<=max(time_range));
        fns = fieldnames(data_struct);
        N_fields = length(fns);
        for i_field = 1:N_fields
            current_field_array = data_struct.(fns{i_field});
            if length(current_field_array) > 1
                valid_struct.(fns{i_field}) = current_field_array(valid_idxs,:);
    
            end
    
        end
        valid_struct.centiSeconds = data_struct.centiSeconds;
        valid_struct.Npoints = length(valid_struct.ROS_Time);
    end

end