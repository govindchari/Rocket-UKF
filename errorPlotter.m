%% Post Processing
r_est = xhatu(1:3,:);
v_est = xhatu(4:6,:);
q_est = xhatu(7:10,:);

r_error = r_true - r_est';
v_error = v_true - v_est';
q_error = q_true - q_est';

for i=1:length(t)
    q_est_norm(i) = norm(q_est(:,i));
    q_est_normalized(i,:) = q_est(:,i)/q_est_norm(i);
    q_error_rot(i,:) = quatmultiply(quatconj(q_true(i,:)),q_est_normalized(i,:));
    attitude_error(i) = rad2deg(2*acos(q_error_rot(i,1)));
end

rxstd=reshape(Pu(1,1,:).^0.5,length(t),1);
rystd=reshape(Pu(2,2,:).^0.5,length(t),1);
rzstd=reshape(Pu(3,3,:).^0.5,length(t),1);
vxstd=reshape(Pu(4,4,:).^0.5,length(t),1);
vystd=reshape(Pu(5,5,:).^0.5,length(t),1);
vzstd=reshape(Pu(6,6,:).^0.5,length(t),1);
q1std=reshape(Pu(7,7,:).^0.5,length(t),1);
q2std=reshape(Pu(8,8,:).^0.5,length(t),1);
q3std=reshape(Pu(9,9,:).^0.5,length(t),1);
q4std=reshape(Pu(10,10,:).^0.5,length(t),1);

%% Error Plotting
figure(10)
errorPlot(t,r_error(:,1),rystd,'X-Position')
figure(11)
errorPlot(t,r_error(:,2),rystd,'Y-Position')
figure(12)
errorPlot(t,r_error(:,3),rzstd,'Z-Position')
figure(13)
errorPlot(t,v_error(:,1),vxstd,'X-Velocity')
figure(14)
errorPlot(t,v_error(:,2),vystd,'Y-Velocity')
figure(15)
errorPlot(t,v_error(:,3),vzstd,'Z-Velocity')
figure(16)
errorPlot(t,q_error(:,1),q1std,'q1')
figure(17)
errorPlot(t,q_error(:,2),q2std,'q2')
figure(18)
errorPlot(t,q_error(:,3),q3std,'q3')
figure(19)
errorPlot(t,q_error(:,4),q4std,'q4')

figure(20)
plot(t,attitude_error,'Linewidth',2)
xlabel('Time (sec)','FontSize',14)
ylabel('Attiude Error (deg)','FontSize',14)
title('Attitude Determination Error','FontSize',14)

function errorPlot(t,z,s,statename)
    plot(t,z,'-r')
    hold on
    plot(t,z+2*s,'b')
    plot(t,z-2*s,'b')
    yline(0,'k','linewidth',2)
    legend(strcat(statename, " Error"),'$2\sigma$ Bounds','interpreter','latex','FontSize',14,'location','southeast')
    title(strcat(statename, " Error"),'FontSize',14)
    xlabel('Time (sec)','FontSize',14)
    ylabel(strcat(statename, " Error"),'FontSize',14)
end

