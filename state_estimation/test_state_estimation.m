global file;

fprintf(file, 'Beginning test_state_estimation\n');

test_init(file);
test_estimator(file);


function estimatorParams = set_up_estimator()
 estimatorParams.llsSeeding = false;
 estimatorParams.batchSamples = 0;
 estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
 estimatorParams.qGain = 1; %Process noise gain for forward prediction
 estimatorParams.initState = [1.5;-1;0;0]; %Constant x-vel init state
 estimatorParams.currentTime = 0;
 [~, estimatorParams] = init_estimator([0, 0], estimatorParams);
end

function test_init(file)
 global lineNum;
 fprintf(file, '  test_init\n');
 estimatorParams.llsSeeding = false;
 estimatorParams.batchSamples = 0;
 estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
 estimatorParams.qGain = 1; %Process noise gain for forward prediction
 estimatorParams.initState = [1.5;-1;0;0]; %Constant x-vel init state
 estimatorParams.currentTime = 0;
 
 % test no seeding
 readings = [0,0];
 [offset, estimatorParams] = init_estimator(readings, estimatorParams);
 test_assert(true, file, lineNum(dbstack), offset, 1);
 test_assert(true, file, lineNum(dbstack), estimatorParams.filter.State, estimatorParams.initState);
 test_assert(true, file, lineNum(dbstack), estimatorParams.filter.StateCovariance, eye(4));
 
 % test seeding
 estimatorParams.llsSeeding = true;
 estimatorParams.batchSamples = 100;
 readings = ones(estimatorParams.batchSamples, 2);
 [offset, estimatorParams] = init_estimator(readings, estimatorParams);
 test_assert(true, file, lineNum(dbstack), offset, estimatorParams.batchSamples + 1);
end

function test_estimator(file)
 global lineNum;
 fprintf(file, '  test_estimator\n');
 estimatorParams = set_up_estimator();
 
 % test negative step size
 time = -1;
 try
  [~, estimatorParams] = state_estimator([nan, nan], time, estimatorParams);
  test_assert(true, file, lineNum(dbstack),  'negative step size', true);
 catch
 end
 
 % test correction
 time = 0.1;
 reading = [1,0];
 [estimate, estimatorParams] = state_estimator(reading, time, estimatorParams);
 test_assert(true, file, lineNum(dbstack), estimate.corrState, estimatorParams.filter.State);
 test_assert(true, file, lineNum(dbstack), estimate.Pcorr, estimatorParams.filter.StateCovariance);
 test_assert(false, file, lineNum(dbstack), estimate.predState, estimate.corrState);
 test_assert(false, file, lineNum(dbstack), estimate.Ppred, estimate.Pcorr);
 test_assert(true, file, lineNum(dbstack), time, estimatorParams.currentTime);
 
 % test prediction
 time = 0.2;
 reading = [nan, nan];
 [estimate, estimatorParams] = state_estimator(reading, time, estimatorParams);
 test_assert(true, file, lineNum(dbstack), all(isnan(estimate.corrState)));
 test_assert(true, file, lineNum(dbstack), all(isnan(estimate.Pcorr), 'all'));
 test_assert(true, file, lineNum(dbstack), estimate.Ppred, estimatorParams.filter.StateCovariance);
 test_assert(true, file, lineNum(dbstack), estimate.predState, estimatorParams.filter.State);
 test_assert(true, file, lineNum(dbstack), time, estimatorParams.currentTime);
end
