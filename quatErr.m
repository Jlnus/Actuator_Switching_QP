function qe = quatErr(q1, q2) % q1^-1 x q2
% q0 q1 q2 q3

q1c = [q1(1);-q1(2:4)]; % conj
q1 = q1c / norm(q1);

qe = [q1(1)*q2(1) - dot(q1(2:4), q2(2:4));
    q1(1)*q2(2:4) + q2(1)*q1(2:4) + cross(q1(2:4),q2(2:4))];
qe = qe/norm(qe);

% if qe(1) < 0 , qe = -qe; end

end
