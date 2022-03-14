sigman_phi = 10;
K_rho = 1;
sigma_rho = 10;

rhodot_d = .1;

x_0 = 0;
y_0 = 0;
phi_0 = 0;

x_s = [0 3 6 10];
y_s = [0 0 3 3];


sim('Steering_Simulation_script')
animate
