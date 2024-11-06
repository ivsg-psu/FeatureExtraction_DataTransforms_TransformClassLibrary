function filled_dataStructure = fcn_DataClean_fillGPSMissingData(dataStructure)



%% Grab GPS units
[~, sensor_names_GPS_Time] = fcn_DataClean_pullDataFromFieldAcrossAllSensors(dataStructure, 'GPS_Time','GPS');

%% Convert LLA to ENU for all GPS units
filled_dataStructure = dataStructure;
for idx_gps_unit = 1:length(sensor_names_GPS_Time)
    GPSUnitName = sensor_names_GPS_Time{idx_gps_unit};
    GPSdataStructure = dataStructure.(GPSUnitName);
    filled_GPSdataStructure = GPSdataStructure;
    GPS_xEast = GPSdataStructure.xEast;
    GPS_yNorth = GPSdataStructure.yNorth;
    GPS_zUp = GPSdataStructure.zUp;
    GPS_DGPS_mode = GPSdataStructure.DGPS_mode;
    GPS_xEast_filled = fillmissing(GPS_xEast,'linear');
    GPS_yNorth_filled = fillmissing(GPS_yNorth,'linear');
    GPS_zUp_filled = fillmissing(GPS_zUp,'linear');
    GPS_DGPS_mode_filled = fillmissing(GPS_DGPS_mode,'next');

    filled_GPSdataStructure.xEast = GPS_xEast_filled;
    filled_GPSdataStructure.yNorth = GPS_yNorth_filled;
    filled_GPSdataStructure.zUp = GPS_zUp_filled;
    filled_GPSdataStructure.DGPS_mode = GPS_DGPS_mode_filled;
    
    filled_dataStructure.(GPSUnitName) = filled_GPSdataStructure;
end

