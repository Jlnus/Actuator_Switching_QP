function T_cmd = PWPFM_thr(filtered_u,PF,t_now)
persistent prev_on

if isempty(prev_on) || t_now==0; prev_on = zeros(2,1); end
 
U_on = PF.U_on;
U_off = PF.U_off; 
deadzone = 0.0001;  % 미세 신호 무시 0.05 * 0.01 / period
 
% 정규화
u_norm = filtered_u / 0.05; % max torque=0.05
u_norm(u_norm < deadzone) = 0;
 
% 슈미트 트리거
for i = 1:2
    if prev_on(i) == 0
        % 이전 OFF 
        if u_norm(i) >= U_on
            prev_on(i) = 1;
        end
    else
        % 이전 ON 
        if u_norm(i) <= U_off
            prev_on(i) = 0;
        end
    end
end

 

T_cmd = 0.05*(prev_on);

end