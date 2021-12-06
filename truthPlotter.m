%% Truth Plotting
figure(1)
plot3(r_true(:,1),r_true(:,2),-r_true(:,3),'LineWidth',2)
xlabel('North','FontSize',14)
ylabel('East','FontSize',14)
zlabel('Up','FontSize',14)
title('Launch Trajectory','FontSize',14)
axis equal
grid on

figure(2)
sgtitle('Position','FontSize',14)
subplot(3,1,1)
plot(t,r_true(:,1),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Distance (m)','FontSize',14)
title('North','FontSize',14)
grid on

subplot(3,1,2)
plot(t,r_true(:,2),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Distance (m)','FontSize',14)
title('East','FontSize',14)
grid on

subplot(3,1,3)
plot(t,-r_true(:,3),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Distance (m)','FontSize',14)
title('Up','FontSize',14)
grid on

figure(3)
sgtitle('Velocity','FontSize',14)
subplot(3,1,1)
plot(t,v_true(:,1),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Speed (m/s)','FontSize',14)
title('North','FontSize',14)
grid on

subplot(3,1,2)
plot(t,v_true(:,2),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Speed (m/s)','FontSize',14)
title('East','FontSize',14)
grid on

subplot(3,1,3)
plot(t,-v_true(:,3),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Speed (m/s)','FontSize',14)
title('Up','FontSize',14)
grid on

figure(4)
sgtitle('Attitude','FontSize',14)
subplot(4,1,1)
plot(t,q_true(:,1),'LineWidth',2)

xlabel('Time (sec)')
title('$q_0$','Interpreter','latex','FontSize',14)
grid on

subplot(4,1,2)
plot(t,q_true(:,2),'LineWidth',2)

xlabel('Time (sec)')
title('$q_1$','Interpreter','latex','FontSize',14)
grid on

subplot(4,1,3)
plot(t,q_true(:,3),'LineWidth',2)

xlabel('Time (sec)')
title('$q_2$','Interpreter','latex','FontSize',14)
grid on

subplot(4,1,4)
plot(t,q_true(:,4),'LineWidth',2)

xlabel('Time (sec)')
title('$q_3$','Interpreter','latex')
grid on

figure(5)
plot(t,qnorm,'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
title('$q_{norm}$','Interpreter','latex')

figure(6)
sgtitle('Angular Rate')
subplot(3,1,1)
plot(t,w_true(:,1),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Rate (rad/s)','FontSize',14)
title('X','FontSize',14)
grid on

subplot(3,1,2)
plot(t,w_true(:,2),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Rate (rad/s)','FontSize',14)
title('Y','FontSize',14)
grid on

subplot(3,1,3)
plot(t,w_true(:,3),'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Rate (rad/s)','FontSize',14)
title('Z')
grid on

figure(7)
plot(t,angle,'LineWidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Angle (deg)','FontSize',14)
title('Angle vs Time','FontSize',14)
grid on

