clear all
type = '2d_RRR';

Fs = 1000;
Ts = 1/Fs;

dataname = 'F0.15.csv'

data = csvread(dataname);

N = size(data,1);
t = (0:N-1)*Ts;
time = N*Ts;

% angle
qref = data(:,9:11)';
q = data(:,17:19)';
qerr = zeros(3,N);

for i = 1:3
    if( q(i,1) > pi )
        q(i,:) = q(i,:) - 2*pi*ones(1,N);
    elseif( q(i,1) < -pi )
        q(i,:) = q(i,:) + 2*pi*ones(1,N);
    end
end
for i = 1:3
    if( qref(i,1) > pi )
        qref(i,:) = qref(i,:) - 2*pi*ones(1,N);
    elseif( qref(i,1) < -pi )
        qref(i,:) = qref(i,:) + 2*pi*ones(1,N);
    end
end

for i=2:N
    qerr(:,i) = q(:,i)-qref(:,i);
end

% velocity
Fd = 100;
dqref = nan(3,N);
ddqref = nan(3,N);
for i = 1:3
    [~,dqref(i,:),ddqref(i,:)] = LPF_3rd(qref(i,:),Fd,Fs);
end

dq = data(:,21:23)';

q_lpf = nan(3,N);
dq_lpf = nan(3,N);
ddq_lpf = nan(3,N);
for i = 1:3
    [q_lpf(i,:),dq_lpf(i,:),ddq_lpf(i,:)] = LPF_3rd(q(i,:),Fd,Fs);
end

% Torque
Ftau = 100;
tau = data(:,25:27)';
tau_lpf = nan(3,N);
for i = 1:3
    tau_lpf(i,:) = LPF_3rd(tau(i,:), Ftau, Fs);
end

ctau = zeros(3,N);
for i = 1:N
    [M,H,G] = get_matrix(type, q(:,i),dq(:,i));
    for j = 1:3
        for k = 1:3
            if k~=j
                ctau(j,i) = ctau(j,i) + M(j,k)*ddq_lpf(k,i);
            end
        end
    end
end

% Translating Vector
C = [0; 3.92; 0];
Vt = nan(3,N);
Vt_norm = nan(1,N);
for k = 1:N
    [J,Jp,U] = fJacobi_q(type,q(:,i));
    Jei = [Jp, U];
    
    Vt(:,k) = C - Jei.'*tau_lpf(:,k);
    Vt_norm(:,k) = norm(Vt(:,k));
end

% Task space
r = zeros(2,N);
for i = 1:N
    r(:,i) = fKinematics(type,q(:,i));
end

dr = zeros(2,N);
dz = zeros(1,N);
drref = zeros(2,N);
drnorm = nan(1,N);
drrefnorm = nan(1,N);
for i = 1:N
    [J,Jp,U] = fJacobi_q(type,q(:,i));
    dr(:,i) = J*dq(:,i);
    
    dz(:,i) = U.'*dq(:,i);
    
    J = fJacobi_minus_q(type,qref(:,i));
    drref(:,i) = J*dqref(:,i);
    
    drnorm(:,i) = norm( dr(:,i) );
    drrefnorm(:,i) = norm(drref(:,i));
end

r_mes = [0; 0.2];
r_err = nan(1,N);
for k = 1:N
    r_err(:,k) = norm( r(:,k) - r_mes );
end

[~,index] = min(r_err);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% draw
FH = 1;
figure(FH)
clf(FH)
r2d = 180/pi;
xarea = [0.1, time];
subplot(3,2,1) % angle
    hold on
    plot(t(1:N-1),q(1,1:N-1)*r2d,'r', t(1:N-1),q(2,1:N-1)*r2d,'g', t(1:N-1),q(3,1:N-1)*r2d,'b')
    plot(t(1:N-1),qref(1,1:N-1)*r2d,'m', t(1:N-1),qref(2,1:N-1)*r2d,'y', t(1:N-1),qref(3,1:N-1)*r2d,'c')
    grid on
    xlim(xarea);
    ylabel('Joint angle [deg]')
    xlabel('time [sec]')
    legend('J1','J2','J3')
    
subplot(3,2,2) % angle error
    hold on
    plot(t(1:N-1),qerr(1,1:N-1)*r2d,'r', t(1:N-1),qerr(2,1:N-1)*r2d,'g', t(1:N-1),qerr(3,1:N-1)*r2d,'b')
    grid on
    xlim(xarea);
    ylabel('Joint angle error [deg]')
    xlabel('time [sec]')
    
subplot(3,2,3) % velocity
    hold on
%     plot(t(1:N-1),dq(1,1:N-1),'r', t(1:N-1),dq(2,1:N-1),'g', t(1:N-1),dq(3,1:N-1),'b')
    plot(t(1:N-1),dq_lpf(1,1:N-1),'r', t(1:N-1),dq_lpf(2,1:N-1),'g', t(1:N-1),dq_lpf(3,1:N-1),'b')
    plot(t(1:N-1),dqref(1,1:N-1),'m', t(1:N-1),dqref(2,1:N-1),'y', t(1:N-1),dqref(3,1:N-1),'c')
    grid on
    xlim(xarea);
    title( strcat('Fc=',num2str(Fd),'[Hz]') )
    ylabel('Joint velocity (LPF) [rad/s]')
    xlabel('time [sec]')
    
subplot(3,2,4) % torque
    plot(t(1:N-1),tau_lpf(1,1:N-1),'r', t(1:N-1),tau_lpf(2,1:N-1),'g', t(1:N-1),tau_lpf(3,1:N-1),'b')
    grid on
    xlim(xarea);
    title( strcat('Fc=',num2str(Ftau),'[Hz]') )
    ylabel('Driving torque (LPF) [Nm]')
    xlabel('time [sec]')
    
subplot(3,2,5) % acceleration
    plot(t(1:N-1),ddq_lpf(1,1:N-1),'r', t(1:N-1),ddq_lpf(2,1:N-1),'g', t(1:N-1),ddq_lpf(3,1:N-1),'b')
    grid on
    xlim(xarea);
    title( strcat('Fc=',num2str(Fd),'[Hz]') )
    ylabel('Joint acceleration (LPF) [rad/s^2]')
    xlabel('time [sec]')
    
subplot(3,2,6) % Coupring torque
    plot(t(1:N-1),ctau(1,1:N-1),'r', t(1:N-1),ctau(2,1:N-1),'g', t(1:N-1),ctau(3,1:N-1),'b')
    grid on
    xlim(xarea);
    ylabel('Coupring torque [Nm]')
    xlabel('time [sec]')


% Ex Task space
FH=3;
figure(FH)
clf(FH)
subplot(3,1,1) % Task space
    plot(t,drnorm,'b', t,drrefnorm,'r')
    xlim(xarea);
    ylim([0, 0.2])
    xlabel('time [sec]')
    ylabel('EE velocity [m/s]')
subplot(3,1,2) % dz
    plot(t,dz,'b')
    xlim(xarea);
    ylim([-1,1]*0.5)
    xlabel('time [sec]')
    ylabel('dz')
subplot(3,1,3) % Translating vector
    plot(t,Vt,'b')
    xlim(xarea);
    ylim([0,8])
    title(strcat( 'Measure time=',num2str(index*Ts),'[s]... |Vt|=',num2str(Vt_norm(1,index)), ' |\tau|=',num2str(norm(tau(:,index))),'[Nm]' ))
    xlabel('time [sec]')
    ylabel('Translating vector')
    
% Animation
FH = 2;
clf(FH)
fRoboAnimation_Figure(FH)
for k = 1:10:N
    hold on 
    fRoboAnimation_Robo(FH,type,q(:,k))
    quiver(r(1,k),r(2,k), Vt(1,k),Vt(2,k), 'AutoScaleFactor',0.1)
    quiver(r(1,index),r(2,index), Vt(1,index),Vt(2,index), 'AutoScaleFactor',0.1, 'Color','r')
    xlim([-0.1,0.2])
    ylim([0, 0.3])
    axis equal
    drawnow
end
% fRoboAnimation(type,FH,q(:,1:N-1), r(:,1:N-1),0)

%%
FH = 5;
figure(FH)
clf(FH)
hold on

plot3(V(1,:),V(2,:),V(3,:),'b')
AH = quiver3(0,0,0, V(1,1),V(2,1),V(3,1),'r', 'AutoScaleFactor',1);

xlabel('C1')
ylabel('C2')
zlabel('C3')

area = 6;
xlim([-1,1]*area)
ylim([-1,1]*area)
zlim([-1,1]*area)

plot3([-area,area],[0,0],[0,0],'k')
plot3([0,0],[-area,area],[0,0],'k')
plot3([0,0],[0,0],[-area,area],'k')

box on
axis square
view([-70,30])

for i = 1:10:N
    title( strcat('time : ',num2str(i*1/1000)) )
    set(AH, 'Udata',V(1,i), 'Vdata',V(2,i), 'Wdata',V(3,i))
    
    drawnow
end