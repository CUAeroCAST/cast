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
 est.Ppred = eye(4);
 est.predState = [0,0];

end

function test_find_sat_pos(file)

end

function test_find_maneuver_pos(file)

end

function test_find_burn_time(file)

end

function test_make_maneuver(file)

end
