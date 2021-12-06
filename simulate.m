%% Parameters

% Initial Conditions
r0 = zeros(3,1);
v0 = [0;0;-0.1];
q0 = [1;0;0;0];
w0 = zeros(3,1);
z0 = [r0;v0;q0;w0];
opt = odeset('RelTol',1e-16,'AbsTol',1e-16,'Events', @apogeeEventFcn);

% Motor Parameters
T = 3500;
tburn = 5.3;

% Vehicle Parameters
CL = 0.5;
CD = 0.5;
cg2cp = 0.3;
m = 45;
R = 0.1524;
L = 3.6576;

%Sensor Noise Parameters
accel_std = 0.1;
gyro_std = 2e-4;
gps_std = 4;

R_accel = diag([ones(3,1)]) * accel_std^2;
R_gyro = diag([ones(3,1)]) * gyro_std^2;
R_gps = diag([ones(3,1)]) * gps_std^2;

% Environmental Parameters
rho = 1.225;
g = 9.81;
vw = [5;8;0];

%Simulation Parameters
dt = 0.01;
tmax = 16;
tspan = linspace(0,tmax,tmax/dt+1);

%% Simulate Flight to Apogee
[t,z] = ode45(@(t,z) eom(t,z,T,tburn,CL,CD,cg2cp,m,R,L,rho,g,vw),tspan,z0,opt);

r_true = z(:,1:3);
v_true = z(:,4:6);
q_true = z(:,7:10);
w_true = z(:,11:13);

for i=1:length(q_true)
    qnorm(i)=norm(q_true(i,:));
    angle(i) = 2*acos(q_true(i,1))*(180/pi);
end
%% Simulate Sensor Readings
for i=1:length(t)
    dz(i,:) = eom(t(i),z(i,:)',T,tburn,CL,CD,cg2cp,m,R,L,rho,g,vw);
    bCi = quat2dcm(q_true(i,:));
    accel(i,:) = bCi*dz(i,4:6)' + (randn(1,3) * sqrt(R_accel))';
    gyro(i,:) = z(i,11:13) + randn(1,3) * sqrt(R_gyro);

    % Generate GPS readings at 10 Hz
    if t(i)*10 == round(t(i)*10)
        gps(i,:) = z(i,1:3) + randn(1,3) * sqrt(R_gps);
    else
        gps(i,:) = 0;
    end
end
%% Apogee Event Function
function [vel,isterminal,direction] = apogeeEventFcn(~,z)
  vel = z(6); % The value that we want to be zero
  isterminal = 1;  % Halt integration 
  direction = 0;   % The zero can be approached from either direction
end
