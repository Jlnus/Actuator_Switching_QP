% 논문 구현
% A Constrained Control Allocation and Tuning Scheme for Hybrid Actuators in Spacecraft (2023)
% RW+Thruster+magnetorquer QP Control Allocation
% -----------------------------------------------------------------------------
% 추력기+자기토커로 모멘텀 덤핑 수행
% 적분기 Runge Kutta 4 버전
% loop : PD -> quadprog -> rk4

clear;

% Simulation Settings
T_end = 60;                 % [s]
dt    = 0.1;               % [s]
N     = round(T_end/dt) + 1;
t     = (0:N-1)*dt;

% Spacecraft & Actuators
P.J = [ 0.0775 0.0005 -0.0002; % [kg m^2]
    -0.0005 0.1067 0.0002;
    0.0002 -0.0002 0.0389];

% RW
P.Jw = 2.3e-5 * eye(4); % [kg m^2]
P.Cw = [1 0 0 1/sqrt(3);
    0 1 0 1/sqrt(3);
    0 0 1 1/sqrt(3)];
T_w_max = 3.2e-3; % [Nm]
target_w_rate = 2000*[1;1;1;-sqrt(3)]*pi/30; % [rad/s]

% Thruster
P.CT = [0 0; 1 -1; 0 0];
T_T_max = 0.05; % def:0.05

% Magnetorquers
T_m_max = 0.92;

% QP param
rho = 1;  P.rho = rho;       % rho*CT*tau_T = u_h + s2
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

% PD control
quat0 = [0.711; 0.319; -0.283; 0.559]; % [qs;qv]
target_quat = [-0.340; -0.610; 0.686; -0.203]; qd = target_quat;
omega0 = [0;-0.00111;0]; % [rad/s]
target_omega = [0;-0.00111;0];


% PD gain
kp = 0.02; % fast
kd = 0.08;
kp = 3.5e-3; kd = 1e-2; % slow

orb.a = 6871e3; orb.e = 1e-5; orb.i = deg2rad(97.6);
orb.Om = deg2rad(80.0); orb.om = 0.0; orb.nu0 = 0.0;

X = zeros(N,11);
tau = zeros(N-1,12);
s1 = zeros(N-1,3);
s2 = zeros(N-1,3);
u = zeros(N-1,3);
u_h = zeros(N-1,3);
q_e_log = zeros(N-1,4);
omega_e_log = zeros(N-1,3);
wheel_rate_log = zeros(N-1,4);
eta_T_log = zeros(N-1,1);

tUTC = datetime(2023,7,10,0,0,0,'TimeZone','UTC');

%% Simulation

X(1,:) = [quat0;omega0;target_w_rate]'; % q, omega, w_rate

for i = 1:N-1
    % mag
    b = get_b_body_from_t_igrf(tUTC+seconds(t(i)), X(i,1:4)', orb);
    blog(i,:) = b;
    bhat = b/norm(b);
    P.CM = -[0 -b(3) b(2); 
             b(3) 0 -b(1);
            -b(2) b(1) 0]*[eye(3), -eye(3)];

    if norm(b)>epsi
        theta = 1;
        P.theta = 1;
    else
        theta = 0;
        P.theta = 0;
    end
    kh  = theta*km + (rho-theta)*kT;

    % PD      
    q_e = quatErr(target_quat, X(i,1:4)');
    omega_e = X(i,5:7)' - target_omega; % current - target

    u(i,:) = -kp*sign(q_e(1))*q_e(2:4) - kd*omega_e; % PD input

    % quadprog
    % Momentum dumping
    w_e = X(i,8:11)' - target_w_rate; % wheel rate error
    u_h(i,:) = -kh * (eye(3) - theta*bhat*bhat') * (P.Cw * (P.Jw * w_e));
    
    % Dynamic weight
    z1 = norm(w_e);
    z2 = norm(q_e(2:4) + kappa*omega_e);
    eta_m = eta_w * (1 + theta*exp(b0/(norm(b)+gamma2))) / (gamma1 + theta*(alpha1*z1 + alpha2*z2));
    eta_T = eta_w / (gamma3 + beta1*(rho-theta)*z1 + beta2*z2);
    % eta_m = 0.1;
    % eta_T = 1e5;

    % QP constraints
    H = 2*diag([eta_w*ones(4,1); eta_T*ones(2,1); eta_m*ones(6,1) ; v1*ones(3,1); v2*ones(3,1)]);
    f = zeros(18,1);

    Aeq = [  -P.Cw, P.CT, theta*P.CM,  -eye(3),  zeros(3);
        zeros(3,4), (rho - theta)*P.CT, theta*P.CM,    zeros(3),   -eye(3)];
    beq = [u(i,:), u_h(i,:)]';

    A = [-eye(12), zeros(12,6); eye(12),  zeros(12,6)];
    B = [T_w_max*ones(4,1); zeros(2,1); zeros(6,1);
        T_w_max*ones(4,1); T_T_max*ones(2,1); T_m_max*ones(6,1)];

    % Solve QP
    opts  = optimoptions('quadprog','Display','off');
    [z,~,exitflag] = quadprog(H,f,A,B,Aeq,beq,[],[],[],opts);

    %
    if isempty(z) || exitflag <= 0
        if i>1, z = tau(i-1,:).'; else, z = zeros(12,1); end
    end

    tau(i,:) = z(1:12)'; % rw+thr+mag
    s1(i,:) = z(13:15)';
    s2(i,:) = z(16:18)';

    % rk4
    [tt, X(i+1,:)] = rk4(@dynamics, X(i,:)', tau(i,:)', t(i), dt, P);
    % X(i+1,:) = X(i,:) + dt*dynamics(X(i,:)', tau(i,:)',P)';

    Om = [ 0   -target_omega.';
        target_omega   -[  0    -target_omega(3)  target_omega(2);
        target_omega(3)  0    -target_omega(1);
        -target_omega(2)  target_omega(1)  0  ] ];
    qd = qd + 0.5*dt*Om*qd;          % (간단 오일러; 더 정확히 하려면 RK4 적용)
    qd = qd / norm(qd);              % 정규화
    % if qd(1) < 0, qd = -qd; end      % 부호 일관(옵션)
    target_quat = qd;                % 최신 qd(t)를 오차계산/PD/QP에 사용

    % logging
    q_e_log(i,:) = sign(q_e(1))*q_e;
    omega_e_log(i,:) = omega_e;
    eta_T_log(i,:) = eta_T;
    eta_m_log(i,:) = eta_m;
    torque_rw_log(i,:) = -(P.Cw*z(1:4))';
    torque_thr_log(i,:) = (P.CT*z(5:6))';
    torque_m_log(i,:) = theta*(P.CM*z(7:12))';
    u_act(i,:) = -P.Cw*z(1:4) + P.CT*z(5:6) + theta*(P.CM*z(7:12));
    u_h_act(i,:) = theta*P.CM*z(7:12) + (rho - theta)*(P.CT * z(5:6));
    z1_log(i) = z1;
    z2_log(i) = z2;
end


%% plots
C2 = colororder('gem');
figure(1);clf;hold on;grid on;C1 = colororder([0 0 1;0.0070 0.3450 0.0540;1 0 0;1 0 1]);
subplot(3,1,1);hold on;title('principal angle error'); grid on; plot(t(1:end-1), 2*acos(q_e_log(:,1))*180/pi);ylim tight;ylabel('deg')
subplot(3,1,2);hold on;title('quaternion error'); grid on;plot(t(1:end-1),q_e_log);legend('q0','q1','q2','q3','Location','best');ylabel('q_e');ylim padded
subplot(3,1,3);hold on;title('omega error'); grid on;plot(t(1:end-1),omega_e_log*180/pi);xlabel('time [s]');ylabel('deg/s');legend('x','y','z','Location','best')
xlabel('time [s]');

figure(2);clf; hold on; grid on; sgtitle('Actuator Torque'); C2(1:3,:) = C1(1:3,:); colororder(C2)
subplot(4,1,1); plot(t(1:end-1),tau(:,1:4)*1000); ylim([-1.9 3.3]);grid on
ylabel('\tau_{w} [mNm]');legend('\tau_{w1}','\tau_{w2}','\tau_{w3}','\tau_{w4}')
subplot(4,1,2); plot(t(1:end-1), tau(:,7:12));legend('m1','m2','m3','m4','m5','m6'); ylabel('\tau_{m} [Am^2]');ylim padded;grid on
subplot(4,1,3); plot(t(1:end-1), tau(:,5:6)*1000);legend('\tau_{t1}','\tau_{t2}');ylabel('\tau_{T} [mNm]');ylim padded;grid on
for i = 1:length(tau)
    taunorm(i,1) = norm(P.Cw*tau(i,1:4)')*1000;
    taunorm(i,2) = norm(P.CM*tau(i,7:12)');
    taunorm(i,3) = norm(P.CT*tau(i,5:6)')*1000;
end
subplot(4,1,4); plot(t(1:end-1), taunorm);legend('||\tau_w||','||\tau_m||','||\tau_t||');ylabel('||\tau||');grid on;xlabel('time [s]');
set(gca, 'YScale','log');

figure(3);clf; colororder('gem')
subplot(3,1,1);hold on;title('torque');plot(t(1:end-1), u_act*1000); plot(t(1:end-1),u*1000,'LineStyle','--','Color','k','LineWidth',0.5);
ylabel('\tau [mNm]');legend('x','y','z','Location','best');ylim padded
subplot(3,1,2);hold on;title('dumping torque');  plot(t(1:end-1), u_h_act*1000) ;plot(t(1:end-1),u_h*1000,'LineStyle','--','Color','k','LineWidth',0.5);
ylabel('\tau_h [mNm]');legend('x','y','z','Location','best'); ylim padded
subplot(3,1,3);title('wheel rate');hold on; plot(t,X(:,8:11)*30/pi); yline([target_w_rate(1), target_w_rate(4)]*30/pi,'LineStyle','--')
xlabel('time [s]'); ylabel('\omega_w [rpm]');legend('w1','w2','w3','w4');ylim([-5500 5500]);


figure(4);clf; hold on; grid on;
subplot(2,1,1);plot(t(1:end-1),eta_m_log);title('eta m');grid on;set(gca, 'YScale','log');ylabel('\eta_m')
subplot(2,1,2);plot(t(1:end-1),eta_T_log);title('eta T');grid on;xlabel('t [s]');ylabel('\eta_T')

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