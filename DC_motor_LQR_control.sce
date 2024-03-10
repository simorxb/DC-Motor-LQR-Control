scheme = "DC_motor_LQR_control.zcos";

// Import model
importXcosDiagram(scheme);
// Parameters for the model
ctx = "m = 0.1; r = 0.05; J = 0.5*m*r^2; b = 0.0000095; kt = 0.0187; R = 0.6; L = 0.35/1000; ke = 0.0191;";

// Execut the string to create variables in the workspace
execstr(ctx);

// Assign context
scs_m.props.context = ctx;

// Compute A, B, C, D matrices
A = [0 1 0; 0 -b/J kt/J; 0 -ke/L -R/L];
B = [0; 0; 1/L];
C = [1 0 0];
D = [0];

// Calculate the controllability matrix
Co = cont_mat(A, B);
// Check if the system is controllable
if rank(Co) == size(A, 1) then
    disp("The system is controllable.");
else
    disp("The system is not controllable.");
end

// Dynamic system for lqr command
sys = syslin("c", A, B, C);

// Various configurations for the optimisation
clear config;

config(1).Q = diag([1 0.1 0.01]); //Weights on states
config(1).R = 0.5; //Weight on input

config(2).Q = diag([1 0.1 0.01]); //Weights on states
config(2).R = 10; //Weight on input

config(3).Q = diag([1 0.5 0.01]); //Weights on states
config(3).R = 0.5; //Weight on input

config(4).Q = diag([1 0.1 0.5]); //Weights on states
config(4).R = 0.5; //Weight on input

// Try all configurations
clear out;
for ctSim = 1:length(config)
    // Find LQR gains
    K = -lqr(sys ,config(ctSim).Q ,config(ctSim).R);
    
    // Find setpoint gain needed
    Kr = 1/((C + D*K)*inv(-A+B*K)*B+D);

    // Simulate
    xcos_simulate(scs_m, 4);
    
    // Store output
    out(ctSim).theta_setpoint = theta_setpoint;
    out(ctSim).theta_out = theta_out;
    out(ctSim).omega_out = omega_out;
    out(ctSim).current_out = current_out;
    out(ctSim).voltage_out = voltage_out;
end

// Draw

v_marg = 0.1;

show_window(1);
subplot(411);
h = plot(out(1).theta_out.time, out(1).theta_out.values, 'b-', out(2).theta_out.time, out(2).theta_out.values, 'g-', out(1).theta_setpoint.time, out(1).theta_setpoint.values, 'r--', 'LineWidth',3);
l = legend("Feedback - Config 1", "Feedback - Config 2", "Setpoint");
l.font_size = 3;
ax=gca();
set(ax, "margins", [0.1, 0.1, v_marg, v_marg]);
set(ax,"grid",[1 1]);
ylabel('Angular position [rad]', 'font_style', 'times bold', 'font_size', 3);

subplot(412);
h = plot(omega_out.time, omega_out.values, 'b-', 'LineWidth',3);
ax=gca();
set(ax, "margins", [0.1, 0.1, v_marg, v_marg]);
set(ax,"grid",[1 1]);
ylabel('Angular speed [rad/s]', 'font_style', 'times bold', 'font_size', 3);

subplot(413);
h = plot(current_out.time, current_out.values, 'b-', 'LineWidth',3);
ax=gca();
set(ax, "margins", [0.1, 0.1, v_marg, v_marg]);
set(ax,"grid",[1 1]);
ylabel('Current [A]', 'font_style', 'times bold', 'font_size', 3);

subplot(414);
h = plot(voltage_out.time, voltage_out.values, 'b-', 'LineWidth',3);
ax=gca();
set(ax, "margins", [0.1, 0.1, v_marg, 0.25]);
set(ax,"grid",[1 1]);
xlabel('Time [s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('Voltage [V]', 'font_style', 'times bold', 'font_size', 3);


