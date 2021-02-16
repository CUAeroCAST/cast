global file;

fprintf(file, 'Beginning test_guidance\n');

test_calc_prob(file);
test_find_sat_pos(file);
test_find_maneuver_pos(file);
test_find_burn_time(file);
test_make_maneuver(file);


function test_calc_prob(file)
 global lineNum;
 fprintf(file, '  test_calc_prob\n');
 est = struct;
 est.predState = [0, 0,0];
 
 % under threshold
  est.Ppred = eye(4) * 1e-2;
 [pdf, ~, ~, ~] = calculate_probability(est);
 test_assert(false, file, lineNum(dbstack), pdf, 0);
 
 % over threshold
 est.Ppred = eye(4);
 [pdf, ~, ~, ~] = calculate_probability(est);
 test_assert(true, file, lineNum(dbstack), pdf, 0);
end

function test_find_sat_pos(file)
 global lineNum;
 fprintf(file, ' test_find_sat_pos\n');
 est = struct;
 est.predState = [0, 0,0];
 est.Ppred = eye(4);

 [x,y] = find_sat_position(est);
 test_assert(true, file, lineNum(dbstack), x, [-3, 3]);
 test_assert(true, file, lineNum(dbstack), y, [-3, 3]);
end

function test_find_maneuver_pos(file)
 fprintf(' test_find_maneuver_position not implemented, pending LUT\n');
end

function test_find_burn_time(file)
 fprintf(' test_find_burn_time not implemented, pending LUT\n');
end

function test_make_maneuver(file)
 fprintf(' test_make_maneuver not implemented, pending LUT\n');
end
