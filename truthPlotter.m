%% Truth Plotting
figure(1)
plot3(r_true(:,1),r_true(:,2),-r_true(:,3),'LineWidth',2)
xlabel('North')
ylabel('East')
zlabel('Up')
axis equal
grid on

figure(2)
sgtitle('Position')
subplot(3,1,1)
plot(t,r_true(:,1),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Distance (m)')
title('North')
grid on

subplot(3,1,2)
plot(t,r_true(:,2),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Distance (m)')
title('East')
grid on

subplot(3,1,3)
plot(t,-r_true(:,3),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Distance (m)')
title('Up')
grid on

figure(3)
sgtitle('Velocity')
subplot(3,1,1)
plot(t,v_true(:,1),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Speed (m/s)')
title('North')
grid on

subplot(3,1,2)
plot(t,v_true(:,2),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Speed (m/s)')
title('East')
grid on

subplot(3,1,3)
plot(t,-v_true(:,3),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Speed (m/s)')
title('Up')
grid on

figure(4)
sgtitle('Attitude')
subplot(4,1,1)
plot(t,q_true(:,1),'LineWidth',2)

xlabel('Time (sec)')
title('$q_0$','Interpreter','latex')
grid on

subplot(4,1,2)
plot(t,q_true(:,2),'LineWidth',2)

xlabel('Time (sec)')
title('$q_1$','Interpreter','latex')
grid on

subplot(4,1,3)
plot(t,q_true(:,3),'LineWidth',2)

xlabel('Time (sec)')
title('$q_2$','Interpreter','latex')
grid on

subplot(4,1,4)
plot(t,q_true(:,4),'LineWidth',2)

xlabel('Time (sec)')
title('$q_3$','Interpreter','latex')
grid on

figure(5)
plot(t,qnorm,'LineWidth',2)
xlabel('Time (sec)')
title('$q_{norm}$','Interpreter','latex')

figure(6)
sgtitle('Angular Rate')
subplot(3,1,1)
plot(t,w_true(:,1),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Rate (rad/s)')
title('X')
grid on

subplot(3,1,2)
plot(t,w_true(:,2),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Rate (rad/s)')
title('Y')
grid on

subplot(3,1,3)
plot(t,w_true(:,3),'LineWidth',2)
xlabel('Time (sec)')
ylabel('Rate (rad/s)')
title('Z')
grid on

figure(7)
plot(t,angle,'LineWidth',2)
xlabel('Time (sec)')
ylabel('Angle (deg)')
title('Angle vs Time')
grid on

