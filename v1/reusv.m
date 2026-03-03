% RW+Thruster QP Control Allocation
% 추력기 모멘텀 덤핑 수행
% loop : PD -> quadprog, rk4

clear;

%% Simulation Settings
T_end = 60;                 % [s]
dt    = 0.01;               % [s]
N     = round(T_end/dt) + 1;
t     = (0:N-1)*dt;
deg2rad = pi/180;

%% Spacecraft & Actuators
P.J = [1249.53 0 0; % [kg m^2]
       0 11055.4 0;
       0 0 11282.4];  

% RW
RW1_ROT = [0 45 0]';%ZYX [deg]
RW2_ROT = [0 0 -45]';%ZYX [deg]
RW3_ROT = [0 -45 0]';%ZYX [deg]
RW4_ROT = [0 0 45]';%ZYX [deg]
RW1_POS = [0.5 0 0]';%XYZ [m]
RW2_POS = [0 0.5 0]';%XYZ [m]
RW3_POS = [-0.5 0 0]';%XYZ [m]
RW4_POS = [0 -0.5 0]';%XYZ [m]

RW1_ROTM = angle2dcm(RW1_ROT(1)*deg2rad,RW1_ROT(2)*deg2rad,RW1_ROT(3)*deg2rad,'ZYX')';
RW2_ROTM = angle2dcm(RW2_ROT(1)*deg2rad,RW2_ROT(2)*deg2rad,RW2_ROT(3)*deg2rad,'ZYX')';
RW3_ROTM = angle2dcm(RW3_ROT(1)*deg2rad,RW3_ROT(2)*deg2rad,RW3_ROT(3)*deg2rad,'ZYX')';
RW4_ROTM = angle2dcm(RW4_ROT(1)*deg2rad,RW4_ROT(2)*deg2rad,RW4_ROT(3)*deg2rad,'ZYX')';

RW1_Axis = RW1_ROTM(:,3);
RW2_Axis = RW2_ROTM(:,3);
RW3_Axis = RW3_ROTM(:,3);
RW4_Axis = RW4_ROTM(:,3);
P.Cw = [RW1_Axis,RW2_Axis,RW3_Axis,RW4_Axis];
P.Jw = 0.151768 * eye(4); % [kg m^2]
T_w_max = 0.2; % [Nm]

% target_w_rate = 4000*[1;1;1;-sqrt(3)]*pi/30; % [rad/s]

% Thruster
RCS1_ROT = [0 90 0]';%ZYX [deg]
RCS2_ROT = [0 -90 0]';%ZYX [deg]
RCS3_ROT = [0 0 -90]';%ZYX [deg]
RCS4_ROT = [0 0 90]';%ZYX [deg]

RCS1_POS = [0.2 0 0.85]';%XYZ [m]
RCS2_POS = [-0.2 0 0.85]';%XYZ [m]
RCS3_POS = [0 0.2 0.85]';%XYZ [m]
RCS4_POS = [0 -0.2 0.85]';%XYZ [m]

RCS1_THR = [-1 0 0]';
RCS2_THR = [1 0 0]';
RCS3_THR = [0 -1 0]';
RCS4_THR = [0 1 0]';

% % % % % % -Z Size % % % % % % % % %
RCS5_ROT = [0 90 0]';%ZYX [deg]
RCS6_ROT = [0 -90 0]';%ZYX [deg]
RCS7_ROT = [0 0 -90]';%ZYX [deg]
RCS8_ROT = [0 0 90]';%ZYX [deg]

RCS5_POS = [0.2 0 -0.72]';%XYZ [m]
RCS6_POS = [-0.2 0 -0.72]';%XYZ [m]
RCS7_POS = [0 0.2 -0.72]';%XYZ [m]
RCS8_POS = [0 -0.2 -0.72]';%XYZ [m]

RCS5_THR = [-1 0 0]';
RCS6_THR = [1 0 0]';
RCS7_THR = [0 -1 0]';
RCS8_THR = [0 1 0]';

% % % % % % +Y Size % % % % % % % % %
RCS9_ROT   = [0 90 0]';%ZYX [deg]
RCS10_ROT = [0 -90 0]';%ZYX [deg]
RCS11_ROT = [0 180 0]';%ZYX [deg]
RCS12_ROT = [0 0 0]';%ZYX [deg]

RCS9_POS   = [0.2 0.85 0]';%ZYX [deg]
RCS10_POS = [-0.2 0.85 0]';%ZYX [deg]
RCS11_POS = [0 0.85 -0.2]';%ZYX [deg]
RCS12_POS = [0 0.85 0.2]';%ZYX [deg]

RCS9_THR  = [-1 0 0]';
RCS10_THR = [1 0 0]';
RCS11_THR = [0 0 1]';
RCS12_THR = [0 0 -1]';

 
% % % % % % -Y Size % % % % % % % % %
RCS13_ROT = [0 90 0]';%ZYX [deg]
RCS14_ROT = [0 -90 0]';%ZYX [deg]
RCS15_ROT = [0 180 0]';%ZYX [deg]
RCS16_ROT = [0 0 0]';%ZYX [deg]

RCS13_POS = [0.2 -0.85 0]';%ZYX [deg]
RCS14_POS = [-0.2 -0.85 0]';%ZYX [deg]
RCS15_POS = [0 -0.85 -0.2]';%ZYX [deg]
RCS16_POS = [0 -0.85 0.2]';%ZYX [deg]

RCS13_THR = [-1 0 0]';
RCS14_THR = [1 0 0]';
RCS15_THR = [0 0 1]';
RCS16_THR = [0 0 -1]';

 
RCS_Pos = [RCS1_POS, RCS2_POS  ,RCS3_POS  ,RCS4_POS,...
                RCS5_POS, RCS6_POS  ,RCS7_POS  ,RCS8_POS,...
                RCS9_POS, RCS10_POS,RCS11_POS,RCS12_POS,...
                RCS13_POS, RCS14_POS,RCS15_POS,RCS16_POS];

P.CT = [RCS1_THR,RCS2_THR,RCS3_THR,RCS4_THR, ...
                      RCS5_THR,RCS6_THR,RCS7_THR,RCS8_THR, ...
                      RCS9_THR,RCS10_THR,RCS11_THR,RCS12_THR, ...
                      RCS13_THR,RCS14_THR,RCS15_THR,RCS16_THR];
T_F_max = 22; % N
% T_T_max = 0.05;  % [Nm]
 
%% QP parameter
rho = 1;         % rho*CT*tau_T = u_h + s2
kT = 0.5;
km = 0.002;
eta_w  = 1.0;           % base weight
beta1  = 1e-5;          %
beta2  = 1e-5;
gamma1 = 1e-2;
gamma2 = 1e-7;
gamma3 = 1e-6;
kappa  = 0.1;
epsi = 1e-7;
b0 = 2.45*1e-5;
alpha1 = 20;
alpha2 = 1e5;
v1 = 2e7;   % s1 weight
v2 = 1e7;   % s1 weight

%% PD control
quat0 = [0.711; 0.319; -0.283; 0.559]; % [qs;qv]
target_quat = [-0.340; -0.610; 0.686; -0.203]; qd = target_quat;
omega0 = [0;-0.00111;0]; % [rad/s]
target_omega = [0;-0.00111;0];


% PD gain
kp = 0.02; % fast
kd = 0.08;  

X = zeros(N,11);
tau = zeros(N-1,6);
s1 = zeros(N-1,3);
s2 = zeros(N-1,3);
u = zeros(N-1,3);
u_h = zeros(N-1,3);
q_e_log = zeros(N-1,4);
omega_e_log = zeros(N-1,3);
wheel_rate_log = zeros(N-1,4);
eta_T_log = zeros(N-1,1); 
filtered_u = [0 ;0];
ts = 0.1; % time constant

%% Simulation

X(1,:) = [quat0;omega0;target_w_rate]'; % q, omega, w_rate

for i = 1:N-1
 
    kh  = rho*kT;

    % ===== PD =====
    q_e = quatErr(target_quat, X(i,1:4)');
    omega_e = X(i,5:7)' - target_omega; % current - target

    u(i,:) = -kp*sign(q_e(1))*q_e(2:4) - kd*omega_e; % PD input  + cross(X(i,5:7)',  P.J*X(i,5:7)')
    
    % ===== quadprog =====
    % Momentum dumping
    w_e = X(i,8:11)' - target_w_rate; % wheel rate error
    u_h(i,:) = -kh * (P.Cw * (P.Jw * w_e));
    
    % Dynamic weight
    z1 = norm(w_e);
    z2 = norm(q_e(2:4) + kappa*omega_e);
    eta_T = eta_w / (gamma3 + beta1*(rho)*z1 + beta2*z2);
    
    % QP constraints
    H = 2*diag([eta_w*ones(4,1); eta_T*ones(2,1); v1*ones(3,1); v2*ones(3,1)]);
    f = zeros(12,1);

    Aeq = [  -P.Cw, P.CT, -eye(3),  zeros(3);
        zeros(3,4), rho*P.CT, zeros(3),   -eye(3)];
    beq = [u(i,:), u_h(i,:)]';

    A = [-eye(6), zeros(6,6); eye(6),  zeros(6,6)];
    B = [T_w_max*ones(4,1); zeros(2,1);  
        T_w_max*ones(4,1); T_T_max*ones(2,1) ];

    % Solve QP
    opts  = optimoptions('quadprog','Display','off');
    [z,~,exitflag] = quadprog(H,f,A,B,Aeq,beq,[],[],[],opts);

    %
    if isempty(z) || exitflag <= 0
        if i>1, z = tau(i-1,:).'; else, z = zeros(12,1); end
    end
    
    PWM_output = PWM_thr(z(5:6), t(i)); % thruster pwm
    filtered_u = filtered_u + dt/ts * (PWM_output - filtered_u);

    tau(i,1:4) = z(1:4)'; % rw
    tau(i,5:6) = filtered_u; % thr
    s1(i,:) = z(7:9)';
    s2(i,:) = z(10:12)';

    % ===== rk4 =====
    [tt, X(i+1,:)] = rk4(@dynamics, X(i,:)', tau(i,:)', t(i), dt, P);
    % X(i+1,:) = X(i,:) + dt*dynamics(X(i,:)', tau(i,:)',P)';

    Om = [ 0   -target_omega.';
        target_omega   -[  0    -target_omega(3)  target_omega(2);
        target_omega(3)  0    -target_omega(1);
        -target_omega(2)  target_omega(1)  0  ] ];
    qd = qd + 0.5*dt*Om*qd;          % 오일러
    qd = qd / norm(qd);              % 정규화
    % if qd(1) < 0, qd = -qd; end      % 부호 일관(옵션)
    target_quat = qd;                % 최신 qd(t)를 오차계산/PD/QP에 사용

    % logging
    q_e_log(i,:) = sign(q_e(1))*q_e;
    omega_e_log(i,:) = omega_e;
    eta_T_log(i,:) = eta_T;
    u_act(i,:) = -P.Cw*z(1:4) + P.CT*z(5:6);
    u_h_act(i,:) = rho*(P.CT * z(5:6));
    z_log(i,:) = z;
    torque_rw(i,:) =  -P.Cw*tau(i,1:4)'; 
    torque_thr(i,:) = P.CT*tau(i,5:6)';
end


%% plots
C2 = colororder('gem');
figure(1);clf;hold on;grid on;C1 = colororder([0 0 1;0.0070 0.3450 0.0540;1 0 0;1 0 1]);
subplot(3,1,1);hold on;title('principal angle error'); grid on; plot(t(1:end-1), 2*acos(q_e_log(:,1))*180/pi);ylim tight;ylabel('deg')
subplot(3,1,2);hold on;title('quaternion error'); grid on;plot(t(1:end-1),q_e_log);legend('q0','q1','q2','q3','Location','best');ylabel('q_e');ylim padded
subplot(3,1,3);hold on;title('omega error'); grid on;plot(t(1:end-1),omega_e_log*180/pi);xlabel('time [s]');ylabel('deg/s');legend('x','y','z','Location','best')
xlabel('time [s]');

figure(2);clf; hold on; grid on; sgtitle('Actuator Torque'); C2(1:3,:) = C1(1:3,:); colororder(C2)
subplot(3,1,1); plot(t(1:end-1),tau(:,1:4)*1000); ylim padded;grid on
ylabel('\tau_{w} [mNm]');legend('\tau_{w1}','\tau_{w2}','\tau_{w3}','\tau_{w4}'); 
subplot(3,1,2); plot(t(1:end-1), tau(:,5:6)*1000);legend('\tau_{t1}','\tau_{t2}');ylabel('\tau_{T} [mNm]');grid on
for i = 1:length(tau)
    taunorm(i,1) = norm(P.Cw*tau(i,1:4)')*1000;
    taunorm(i,3) = norm(P.CT*tau(i,5:6)')*1000;
end
subplot(3,1,3);hold on; 
plot(t(1:end-1), taunorm(:,3),'r');
plot(t(1:end-1), taunorm(:,1),'b',LineWidth=2);
legend('||\tau_t||','||\tau_w||');ylabel('||\tau||');grid on;xlabel('time [s]');
% set(gca, 'YScale','log');

figure(3);clf; colororder('gem')
subplot(3,1,1);hold on;title('torque');plot(t(1:end-1), u_act*1000); plot(t(1:end-1),u*1000,'LineStyle','--','Color','k','LineWidth',0.5);
ylabel('\tau [mNm]');legend('x','y','z','Location','best');ylim padded
subplot(3,1,2);hold on;title('dumping torque');  plot(t(1:end-1), u_h_act*1000) ;plot(t(1:end-1),u_h*1000,'LineStyle','--','Color','k','LineWidth',0.5);
ylabel('\tau_h [mNm]');legend('x','y','z','Location','best'); ylim padded
subplot(3,1,3);title('wheel rate');hold on; plot(t,X(:,8:11)*30/pi); yline([target_w_rate(1), target_w_rate(4)]*30/pi,'LineStyle','--')
xlabel('time [s]'); ylabel('\omega_w [rpm]');legend('w1','w2','w3','w4');ylim([-5500 5500]);

figure(4);clf; colororder('gem')
subplot(3,1,1);hold on;title('Wheel torque');plot(t(1:end-1),torque_rw*1000);
ylabel('\tau_{rw} [mNm]');legend('x','y','z');ylim padded
subplot(3,1,2);hold on;title('Thruster torque');  plot(t(1:end-1), torque_thr*1000);
ylabel('\tau_T [mNm]');legend('x','y','z'); ylim padded
subplot(3,1,3);title('wheel rate');hold on; plot(t,X(:,8:11)*30/pi); yline([target_w_rate(1), target_w_rate(4)]*30/pi,'LineStyle','--')
xlabel('time [s]'); ylabel('\omega_w [rpm]');legend('w1','w2','w3','w4');ylim([-5500 5500]);


% figure(5);clf; hold on; grid on; title('s1: 자세제어')
% plot(t(1:end-1),s1); legend('x','y','z')
%
% figure(6);clf; hold on; grid on; title('s2: 모멘텀 덤핑')
% plot(t(1:end-1),s2); legend('x','y','z')
%  
% 
% figure(111);clf;plot(t(1:end-1),blog);legend('x','y','z');xlabel("t [s]"), ylabel("magnetic field [ T ]");title("Local external magnetic field")
% 
% figure(7)
% plot(t(1:end-1),torque_m_log)