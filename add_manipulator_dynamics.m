function [robot] = add_manipulator_dynamics(robot, coms, masses, Is, g0)
%ADD MANIPULATOR DYNAMICS Adds symbolic expressions needed for the manipulator dynamics.

    robot.coms = coms;
    robot.masses = masses;
    robot.Is = Is;
    robot.g0 = g0;
    n_links = size(robot.DHParams, 1);

    % Centers of mass geometric jacobian
    Jp = cell(n_links,1);
    for i = 1:n_links
        pcom = robot.T{i}*[robot.coms{i}; 1];
        Jp{i} = subs(robot.jacobian, robot.point, pcom(1:3));
        Jp{i} = [Jp{i}(:,1:i) zeros(6,n_links-i)];
        Jp{i} = simplify(Jp{i});
    end
    robot.Jp = Jp;

    %% Inertia matrix, gravity torque and centrifugal/coriolis tensor
    B = zeros(n_links);
    g = zeros(n_links,1);
    for i = 1:n_links
        Jpos = robot.Jp{i}(1:3,:);
        Jor = robot.Jp{i}(4:6,:);
        I = robot.T{i}(1:3,1:3) * robot.Is{i} * robot.T{i}(1:3,1:3)';
        B = B + simplify(robot.masses(i)*Jpos'*Jpos + Jor'*I*Jor);
        g = g - (robot.masses(i)*robot.g0'*Jpos)';
    end 
    B = simplify(B);
    robot.B = B;
    robot.g = g;
    
    Bder = arrayfun(@(i) diff(B, robot.q(i)), 1:n_links, 'un', 0);
    Bder = cat(3, Bder{:});
    C = 0.5 * (Bder + permute(Bder,[1 3 2]) - permute(Bder,[3 1 2]));  % Bder{k}(i,j) + Bder{j}(i,k) - Bder{i}(j,k)
    C = simplify(C); % may be avoided ?
    robot.C = C;

    %% Inertia matrix inverse
    % Binv = inv(rewrite(B,'exp'));
    % Binv = simplify(rewrite(Binv,'exp')); % 40 min for simple 4-joint robot :O
    % robot.Binv = Binv; %matlabFunction(Binv);
    
    
    %% Analytical jacobian derivative
    % generate symbolic time-dependent q vector
    symstr = '[';
    for i = 1:n_links
        symstr = [symstr 'q' num2str(i) '(t); '];
    end
    symstr = [symstr ']'];
    qt = str2sym(symstr);
    robot.qt = qt;
    % substitute q with q(t) and differentiate
    time_dependant_J = subs(Ja, q, qt);
    Ja_dot = diff(time_dependant_J, sym('t'));
    % substitute diff(q,t) with q_dot
    q_dot = sym('q_dot', [n_links,1]);
    Ja_dot = subs(Ja_dot, diff(qt, sym('t')), q_dot);
    % substitute back q(t) with q
    Ja_dot = subs(Ja_dot, qt, q);
    robot.q_dot = q_dot;
    robot.Ja_dot = Ja_dot;
    
    % Ba = inv(Ja*Binv*Ja');
end