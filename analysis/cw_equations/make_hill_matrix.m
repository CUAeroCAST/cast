function [hill_matrix] = make_hill_matrix(n, t)
 RR = [4 - 3 * n * t,          0,                        0;
       6 * sin(n * t) - n * t, 1,                        0;
       0,                      0,                        cos(n * t)];
      
 RV = [sin(n * t) / n,           2 * (1 - cos(n * t)) / n,         0;
       2 * (cos(n * t) - 1) / n, (4 * sin(n * t) - 3 * n * t) / n, 0;
       0,                        0,                                sin(n * t) / n];

 VR = [3 * n * sin(n * t),       0, 0;
       6 * n * (cos(n * t) - 1), 0, 0;
       0,                        0, -n * sin(n * t)];
      
 VV = [cos(n * t),      2 * sin(n * t),     0;
       -2 * sin(n * t), 4 * cos(n * t) - 3, 0;
       0,               0,                  cos(n * t)];
   
 hill_matrix = [RR , RV;
               VR , VV];
end