clc
clear
close all

%% Part 1
%save the data as step(diff_id,poly_coeffs,xyz_id,step_id)
%quinticpoly(t0,tf,q0,qf,qd0,qdf,qdd0,qddf)
% step 1 (0,0,0) to (0,0,1) 0-5s
step(1,:,1,1) = quinticpoly(0,5,0,0,0,0,0,0);%x
step(1,:,2,1) = quinticpoly(0,5,0,0,0,0,0,0);%y
step(1,:,3,1) = quinticpoly(0,5,0,1,0,0,0,0);%z
% step 2 (0,0,1) to (1,0,1) 5-20s
step(1,:,1,2) = quinticpoly(5,20,0,1,0,0,0,0);
step(1,:,2,2) = quinticpoly(5,20,0,0,0,0,0,0);
step(1,:,3,2) = quinticpoly(5,20,1,1,0,0,0,0);
% step 3 (1,0,1) to (1,1,1) 20-35s
step(1,:,1,3) = quinticpoly(20,35,1,1,0,0,0,0);
step(1,:,2,3) = quinticpoly(20,35,0,1,0,0,0,0);
step(1,:,3,3) = quinticpoly(20,35,1,1,0,0,0,0);
% step 4 (1,1,1) to (0,1,1) 35-50s
step(1,:,1,4) = quinticpoly(35,50,1,0,0,0,0,0);
step(1,:,2,4) = quinticpoly(35,50,1,1,0,0,0,0);
step(1,:,3,4) = quinticpoly(35,50,1,1,0,0,0,0);
% step 5 (0,1,1) to (0,0,1) 50-65s
step(1,:,1,5) = quinticpoly(50,65,0,0,0,0,0,0);
step(1,:,2,5) = quinticpoly(50,65,1,0,0,0,0,0);
step(1,:,3,5) = quinticpoly(50,65,1,1,0,0,0,0);

for step_idx = 1:5
    for xyz_idx = 1:3
        for diff_idx = 2:3
            diff_coeffs = polyder(step(diff_idx - 1,:,xyz_idx,step_idx));
            step(diff_idx,:,xyz_idx,step_idx) = [zeros(1,6-size(diff_coeffs,2)) diff_coeffs];
        end
    end
end

ts = 0;
tf = 65;
sample_rate = 100; %Hz
t = linspace(ts,tf,(tf-ts)*sample_rate+1);
%Save trajectory data as trajectory_data(diff_id, data, xyz_idx)
trajectory_data = zeros(3,size(t,2),xyz_idx);
for xyz_idx = 1:3
    for diff_idx = 1:3
        for time_idx = 1:size(t,2)
            step_idx = 0;
            if t(time_idx) >= 0 && t(time_idx) < 5
                step_idx = 1;
            elseif t(time_idx) >= 5 && t(time_idx) < 20
                step_idx = 2;
            elseif t(time_idx) >= 20 && t(time_idx) < 35
                step_idx = 3;
            elseif t(time_idx) >= 35 && t(time_idx) < 50
                step_idx = 4;
            elseif t(time_idx) >= 50
                step_idx = 5;
            end
            trajectory_data(diff_idx,time_idx,xyz_idx) = polyval(step(diff_idx,:,xyz_idx,step_idx),t(time_idx));
        end
    end
end

fig = figure("Name","Trajectory Position in 3D");
plot3(trajectory_data(1,:,1),trajectory_data(1,:,2),trajectory_data(1,:,3));
xlabel("x");
ylabel("y");
zlabel("z");
title("Quadrotor Desired Trajectory");

currentFolder = pwd;
mkdir('plots')
saveas(fig,fullfile(currentFolder,'plots','TrajectoryPositionPlot3D.png'));

%Position plots
fig = figure("Name","Trajectory Positions");
subplot(3,1,1)
plot(t, trajectory_data(1,:,1));
xlabel("t(s)",'Interpreter','latex');
ylabel("x position(m)",'Interpreter','latex');
title("Quadrotor position in x direction");

subplot(3,1,2)
plot(t, trajectory_data(1,:,2));
xlabel("t(s)",'Interpreter','latex');
ylabel("y position(m)",'Interpreter','latex');
title("Quadrotor position in y direction");

subplot(3,1,3)
plot(t, trajectory_data(1,:,3));
xlabel("t(s)",'Interpreter','latex');
ylabel("z position(m)",'Interpreter','latex');
title("Quadrotor position in z direction");

saveas(fig,fullfile(currentFolder,'plots','TrajectoryPositionPlot.png'));

%Velocity plots
fig = figure("Name","Trajectory Velocities");
subplot(3,1,1)
plot(t, trajectory_data(2,:,1));
xlabel("t(s)",'Interpreter','latex');
ylabel("x velocity(m/s)",'Interpreter','latex');
title("Quadrotor velocity in x direction");

subplot(3,1,2)
plot(t, trajectory_data(2,:,2));
xlabel("t(s)",'Interpreter','latex');
ylabel("y velocity(m/s)",'Interpreter','latex');
title("Quadrotor velocity in y direction");

subplot(3,1,3)
plot(t, trajectory_data(2,:,3));
xlabel("t(s)",'Interpreter','latex');
ylabel("z velocity(m/s)",'Interpreter','latex');
title("Quadrotor velocity in z direction");

saveas(fig,fullfile(currentFolder,'plots','TrajectoryVelocityPlot.png'));

%Acceleration plots
fig = figure("Name","Trajectory Accelerations");
subplot(3,1,1)
plot(t, trajectory_data(3,:,1));
xlabel("t(s)",'Interpreter','latex');
ylabel("x acceleration($m/s^2$)",'Interpreter','latex');
title("Quadrotor acceleration in x direction");

subplot(3,1,2)
plot(t, trajectory_data(3,:,2));
xlabel("t(s)",'Interpreter','latex');
ylabel("y acceleration($m/s^2$)",'Interpreter','latex');
title("Quadrotor acceleration in y direction");

subplot(3,1,3)
plot(t, trajectory_data(3,:,3));
xlabel("t(s)",'Interpreter','latex');
ylabel("z acceleration($m/s^2$)",'Interpreter','latex');
title("Quadrotor acceleration in z direction");

saveas(fig,fullfile(currentFolder,'plots','TrajectoryAccelerationPlot.png'));

%Dynamic model
syms x y z phi theta psi 'real' %generalized coordinates
syms u1 u2 u3 u4 'real' %control inputs
syms w1 w2 w3 w4 'real' %rotor speed
syms g m l Ix Iy Iz Ip kF kM wmax wmin 'real' 
param_list = [g m l Ix Iy Iz Ip kF kM wmax wmin];
param_value = [9.81,...                 %gravity constant
               27*10^(-3),...           %m (kg)
               46*10^(-3),...           %l (m)
               16.571710*10^(-6),...    %Quadrotor inertia along x-axis (kg*m^2)
               16.571710*10^(-6),...    %Quadrotor inertia along y-axis (kg*m^2)
               29.261652*10^(-6),...    %Quadrotor inertia along z-axis (kg*m^2)
               12.65625*10^(-8),...     %Propeller moment of inertia    (kg*m^2)
               1.28192*10^(-8),...      %Propeller thrust factor        (N*s^2)
               5.964552*10^(-3),...     %Propeller moment factor        (m)
               2618,...                 %Rotor maximum speed            (rad/s)
               0];                      %Rotor minimum speed            (rad/s)

%allocation matrix
AM = [1/(4*kF), -sqrt(2)/(4*kF*l), -sqrt(2)/(4*kF*l), -1/(4*kM*kF);
      1/(4*kF), -sqrt(2)/(4*kF*l), sqrt(2)/(4*kF*l), 1/(4*kM*kF);
      1/(4*kF), sqrt(2)/(4*kF*l), sqrt(2)/(4*kF*l), -1/(4*kM*kF);
      1/(4*kF), sqrt(2)/(4*kF*l), -sqrt(2)/(4*kF*l), 1/(4*kM*kF)];

%equations of motion
syms x_dot y_dot z_dot x_ddot y_ddot z_ddot 'real'
syms phi_dot theta_dot psi_dot phi_ddot theta_ddot psi_ddot 'real'

omega = w1-w2+w3-w4;
eq1 = x_ddot == 1/m *(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*u1;
eq2 = y_ddot == 1/m *(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*u1;
eq3 = z_ddot == 1/m *(cos(phi)*cos(theta))*u1 - g;
eq4 = phi_ddot == theta_dot*psi_dot*(Iy-Iz)/Ix - Ip/Iy*omega*theta_dot + 1/Ix*u2;
eq5 = theta_ddot == phi_dot*psi_dot*(Iz-Ix)/Iy + Ip/Iy*omega*phi_dot +1/Iy*u3;
eq6 = psi_ddot == phi_dot*theta_dot*(Ix-Iy)/Iz + 1/Iz*u4;

%Rearrang the EOMs
% eq1 = u1 == solve(eq1,u1);
% eq2 = u1 == solve(eq2,u1);
% eq3 = u1 == solve(eq3,u1);
% eq4 = u2 == solve(eq4,u2);
% eq5 = u3 == solve(eq5,u3);
% eq6 = u4 == solve(eq6,u4);

input_var_list = [u1, u2, u3, u4];
input_var_zeros = zeros(size(input_var_list));
dq_var_list = [x_dot, y_dot, z_dot, phi_dot theta_dot psi_dot];
dq_var_zeros = zeros(size(dq_var_list));
ddq_var_list = [x_ddot, y_ddot, z_ddot, phi_ddot theta_ddot psi_ddot];
ddq_var_zeros = zeros(size(ddq_var_list));
eqs = [eq1,eq2,eq3,eq4,eq5,eq6];
% M_eqs = subs(eqs, [input_var_list, dq_var_list, g], [input_var_zeros, dq_var_zeros, 0]);
% C_eqs = subs(eqs, [input_var_list, ddq_var_list, g], [input_var_zeros, ddq_var_zeros, 0]);
% g_eqs = subs(eqs, [input_var_list, dq_var_list, ddq_var_list], [input_var_zeros, dq_var_zeros, ddq_var_zeros]);
% M = equationsToMatrix(M_eqs,ddq_var_list);
% C = rhs(C_eqs)';
% G = rhs(g_eqs)';

f_eqs = subs(eqs, [input_var_list],[input_var_zeros]);
fx = rhs(f_eqs);
gx = rhs(eqs) - fx;
fx = fx'
gx = gx'

