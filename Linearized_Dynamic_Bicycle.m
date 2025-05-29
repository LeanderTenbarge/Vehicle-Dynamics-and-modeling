%Linearized Dynamic Bicycle Model via a state space representation. 
%Leander Tenbarge

%Parameters (resembling a passenger van):
m = 1500;         % Vehicle mass (kg)
Iz = 2250;         % Yaw moment of inertia (kg*m^2)
Lf = 1.2;             % Distance from CG to front axle (m)
Lr = 1.6;             % Distance from CG to rear axle (m)
Cf = 80000;      % Front cornering stiffness (N/rad)
Cr = 80000;      % Rear cornering stiffness (N/rad)
vx = 15;             % Longitudinal velocity (m/s)

% State Space Representation 
A = [ -((Cf+Cr)/(m*vx))        , -((Cf*Lf-Cr*Lr)/(m*vx))         ,0, 0,0;
        -((Cf*Lf-Cr*Lr)/(Iz*vx)), -((Cf*Lf^2+Cr*Lr^2)/(Iz*vx)),0,0,0;
        0                                    ,1                                               ,0,0,0;
        0                                    ,0                                               ,0,0,0;
        1                                     ,0                                              ,vx,0,0];
B = [Cf/m;
        Cf*Lf/Iz;
        0;
        0;
        0;];
C = eye(5);
D = zeros(5,1);
sys = ss(A,B,C,D);

% Running the Simulation
t = 0:0.01:5;                % Time vector (0 to 5 seconds);
delta = zeros(size(t));
for i = 1:length(t)
    delta(i) = .05*sin(t(i)/.5)*exp(-.5*t(i));  % Define delta as sin(t/2)
end


% Simulation the Response of the function:
[y, t, x] = lsim(sys, delta, t);



% Plot selected states (e.g., lateral velocity, yaw rate, heading):
figure;
title('State Response to Sinusoidal Input');
subplot(6,1,1);
plot(t, delta); 
ylabel('Steering input (rad)');
xlabel('Time (s)');

subplot(6,1,2);
plot(t, x(:,1)); 
ylabel('Lateral Velocity (m/s)');
xlabel('Time (s)');

subplot(6,1,3);
plot(t,x(:,2));
ylabel("Yaw rate (rad/s)")
xlabel('Time (s)');

subplot(6,1,4);
plot(t, x(:,3)); 
ylabel('Heading \psi (rad)'); 
xlabel('Time (s)');

subplot(6,1,5);
plot(t,x(:,4));
ylabel('X position (m)')

xlabel('Time (s)');
subplot(6,1,5);
plot(t,x(:,5));
ylabel('Y position (m)')
xlabel('Time (s)');





