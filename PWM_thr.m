function T_cmd = PWM_thr(u, t_now)
persistent prev_on

if isempty(prev_on); prev_on = zeros(2,1); end

freq = 5;           % Hz   
period = 1/freq;    % 주기
amplitude   = 1.0;  % 진폭 (0~1 범위 사용)
deadzone = 0.0001;  % 미세 신호 무시 0.05 * 0.01 / period
hyst     = 0.002;
 
% 정규화
u_norm = u / 0.05; % max torque=0.05
u_norm(u_norm < deadzone) = 0;

% 삼각파 carrier
t_mod = mod(t_now, period);
carrier = amplitude * abs(1 - 2*(t_mod / period));
 
for i = 1:2
    if prev_on(i) == 0
        % 이전 OFF 
        if u_norm(i) > (carrier + hyst)
            prev_on(i) = 1;
        end
    else
        % 이전 ON 
        if u_norm(i) < (carrier - hyst)
            prev_on(i) = 0;
        end
    end
end

if prev_on(1)==1 && prev_on(2)==1
    if u_norm(1) >= u_norm(2)
        prev_on(2) = 0;
    else
        prev_on(1) = 0;
    end
end
  
T_cmd = 0.05*(prev_on);

end