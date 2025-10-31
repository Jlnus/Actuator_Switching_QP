
function b_B = get_b_body_from_t_igrf(tUTC, q_BI, orb)
% IGRF-13 
    muE = 3.986004418e14; % 지구 중력상수
    
    n  = sqrt(muE/orb.a^3);
    nu = wrapToPi(orb.nu0 + n*seconds(tUTC - datetime(2023,7,10,0,0,0,'TimeZone','UTC')));
    [rECI, ~] = perifocal2eci(orb.a,orb.e,orb.i,orb.Om,orb.om,nu,muE); % 궤도 -> ECI
    datev = datevec(tUTC);
    
    % ECI->LLA  
    lla = eci2lla(rECI', datev);
    [lat_deg, lon_deg, h_m] = deal(lla(1), lla(2), lla(3));

    % IGRF: NED [nT] -> [T]
    Bned = igrfmagm(h_m, lat_deg, lon_deg, decyear(datev),13); %nT
    b_ned = 1e-9 * [Bned(1); Bned(2); Bned(3)]; % T

    % NED->ECEF, ECEF->ECI, ECI->BODY
    b_ecef = ned2ecef_vec(b_ned, deg2rad(lat_deg), deg2rad(lon_deg));
    R_ECEF_ECI = dcmECItoECEF_GMST(tUTC); % ECI -> ECEF
    b_eci  = R_ECEF_ECI.' * b_ecef; % ECEF -> ECI
    R_BI   = dcmBodyFromInertial(q_BI); 
    b_B    = R_BI * b_eci; % ECI -> body
end

function R = dcmBodyFromInertial(q)
    q = q(:); qs=q(1); qx=q(2); qy=q(3); qz=q(4);
    R = [1-2*(qy^2+qz^2),   2*(qx*qy+qs*qz),   2*(qx*qz-qs*qy);
         2*(qx*qy-qs*qz),   1-2*(qx^2+qz^2),   2*(qy*qz+qs*qx);
         2*(qx*qz+qs*qy),   2*(qy*qz-qs*qx),   1-2*(qx^2+qy^2)];
end

function [rECI, vECI] = perifocal2eci(a,e,i,RAAN,argp,nu,mu)
    p   = a*(1 - e^2);
    r_pf = (p/(1+e*cos(nu))) * [cos(nu); sin(nu); 0];
    v_pf = sqrt(mu/p) * [-sin(nu); e+cos(nu); 0];
    R = Rz(RAAN)*Rx(i)*Rz(argp);
    rECI = R * r_pf; vECI = R * v_pf;
end

function R = Rx(a), ca=cos(a); sa=sin(a); R=[1 0 0; 0 ca -sa; 0 sa ca]; end
function R = Rz(a), ca=cos(a); sa=sin(a); R=[ca -sa 0; sa ca 0; 0 0 1]; end

function R = dcmECItoECEF_GMST(tUTC)
    jd  = juliandate(tUTC);
    T   = (jd - 2451545.0)/36525.0;
    gmst_sec = 67310.54841 + (876600*3600 + 8640184.812866)*T + 0.093104*T^2 - 6.2e-6*T^3;
    gmst = deg2rad( mod(gmst_sec/240, 360) );
    R = Rz(gmst).'; % ECI->ECEF
end

function b_ecef = ned2ecef_vec(b_ned, lat, lon)
    sl=sin(lat); cl=cos(lat); slon=sin(lon); clon=cos(lon);
    % ENU->ECEF
    R_ecef_enu = [-slon,           -sl, clon*cl; ...
                   clon,           -sl, slon*cl; ...
                   0,               cl,      0  ];
    % NED->ENU
    R_enu_ned  = [0 1 0; 1 0 0; 0 0 -1];
    b_ecef = R_ecef_enu * (R_enu_ned * b_ned);
end


% function a = etam()
% -------- Parameters (논문 Fig.3과 동일) --------
% z1 = 0.1; 
% z2 = 0.005;
% b0 = 2e-5;       % [T] = 20 μT
% alpha1 = 5; 
% alpha2 = 500; 
% gamma1 = 0.01; 
% gamma2 = 1e-7;   % [T] = 0.1 μT
% eta_w  = 1;
% epsB   = 1e-7;   % [T]
% theta  = 1;      % 대부분 구간에서 theta=1
% 
% % -------- Independent variable: |b| in μT --------
% b_uT = linspace(0.1, 50, 1000);   % 0.1~50 μT
% b    = b_uT * 1e-6;               % [T]
% 
% % -------- eta_m(|b|) (논문 식 기반) --------
% eta_m = eta_w * (1 + theta * exp(b0 ./ (b + gamma2))) ./ ...
%               (gamma1 + theta * (alpha1*z1 + alpha2*z2));
% 
% % -------- Plot (y-axis log scale) --------
% figure(111); clf; hold on; grid on;
% semilogy(b_uT, eta_m, 'k', 'LineWidth', 1.5);   % y축 로그
% xlim([0 50]);
% ylim([1e-2 1e10]);
% xlabel('||b|| [\muT]', 'Interpreter','tex');
% ylabel('\eta_m', 'Interpreter','tex');
% title('\eta_m vs. magnetic field strength ||b|| (log scale)');
% set(gca, 'YScale','log','FontName','Times', 'FontSize',12);
% end