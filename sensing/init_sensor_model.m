function [sensorScenario] = init_sensor_model(relativeOrbit, timeVec,...
                                              sensorParams, targetParams)
%INIT_SENSOR_MODEL This function creates a parametric sensor model,
%returning the scenario. 
 sensorScenario = trackingScenario('UpdateRate',sensorParams.samplingRate);
 sensorScenario.StopTime = timeVec(end);
 ego = platform(sensorScenario,'Trajectory', ...
                kinematicTrajectory('Position',[0 0 0],...
                'AngularVelocitySource', 'Property',...
                'AngularVelocity',[0 0 sensorParams.scanRate*2*pi]));
 target = platform(sensorScenario,'Trajectory',...
                   waypointTrajectory(relativeOrbit, timeVec));
 target.Mesh = targetParams.Mesh;
 target.Dimensions = targetParams.Dimensions;

 switch (sensorParams.sensorType)
  case 'Lidar'
   sensorOpts = {'UpdateRate',sensorParams.samplingRate, ...
                 'MountingLocation',[0,0,0], ...
                 'MountingAngles',[0,0,0], ...
                 'MaxRange',sensorParams.maxRange, ...
                 'RangeAccuracy',sensorParams.rangeAccuracy, ...
                 'AzimuthResolution',sensorParams.beamDivergence, ...
                 'ElevationResolution',sensorParams.beamDivergence,...
                 'AzimuthLimits',sensorParams.beamLimits,  ...
                 'ElevationLimits',sensorParams.beamLimits, ...
                 'HasNoise',true, ...
                 'HasINS',true};
   sensor = monostaticLidarSensor(1,sensorOpts{:});
  case 'radar'        
    scanRate = sensorParams.RPM*360/60;
    sensorOpts = {'No Scanning', 'DirectionCoordinates',...
                  'Sensor Rectangular', ...
                  'UpdateRate',sensorParams.samplingRate, ...
                  'MountingLocation', [0 0 0], ...
                  'MountingAngles', [0 0 0],...
                  'MaxMechanicalScanRate',scanRate, ...
                  'FieldOfView',sensorParams.FOV, ...
                  'HasRangeAmbiguities',true, ...
                  'MaxUnambiguousRange',sensorParams.maxRange,...
                  'AzimuthResolution',sensorParams.FOV(1),...
                  'ElevationResolution',sensorParams.FOV(2),...
                  'RangeResolution',sensorParams.rangeResolution,...
                  'Bandwidth',sensorParams.bandWidth,...
                  'CenterFrequency',sensorParams.centerFreq,...
                  'Sensitivity',sensorParams.receiverSensitivity,...
                  'HasNoise',true,...
                  'HasINS',true};
    sensor = monostaticRadarSensor(1,sensorOpts{:});        
   case 'Infrared'        
    sensorOpts = {'No Scanning', 'DirectionCoordinates',...
                  'UpdateRate',sensorParams.samplingRate, ...
                  'MountingLocation', [0 0 0], ...
                  'MountingAngles', [0 0 0],...
                  'MaxMechanicalScanRate',sensorParams.scanRate, ...
                  'FieldOfView',sensorParams.FOV, ...
                  'LensDiameter',sensorParams.lensDiameter,...
                  'FocalLength',sensorParams.focalLength,...
                  'AzimuthResolution',sensorParams.FOV(1),...
                  'ElevationResolution',sensorParams.FOV(2),...
                  'HasNoise',true,...
                  'HasINS',true};
    sensor = irSensor(1,sensorOpts{:});
   otherwise
    fprintf('init_sensor_model switch failed\n');
 end
 ego.Sensors = sensor;
end

