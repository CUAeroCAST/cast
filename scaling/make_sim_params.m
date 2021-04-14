function params = make_sim_params()
 params.angle = 45;
 params.time = 30;
 params.muEarth = 398600;
 params.rate = 10;
 params.ebounds = [-1e-3, 1e-3];
 params.abounds = [-1e-3, 1e-3];
 params.res = 1e-4;
 params.acc = 0.01;
 params.mass = 850;
 
end