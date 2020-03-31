function [robot] = get_manipulator(DHParams, jointTypes, coms, masses, Is, g0, base)
%ROBOT Symbolically computes some matrixes relative to the specified robot.

    %% Input parsing

    robot = struct();
    robot.fixedDHParams = DHParams;
    robot.jointTypes = jointTypes;
    robot.coms = coms;
    robot.masses = masses;
    robot.Is = Is;
    robot.g0 = g0;
    robot.base = base;
    n_links = size(DHParams, 1);
    q = sym('q',[n_links,1]);
    robot.q = q;

    jointTypes = strrep(jointTypes, 'p', '3');
    jointTypes = strrep(jointTypes, 'r', '4');
    jointTypes = arrayfun(@(x) str2num(x), jointTypes);

    dh = DHParams + diag(q)*ind2vec(jointTypes)';
    robot.dh = dh;

    %% Transformation matrixes

    Rz = @(th) [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
    Rx = @(al) [1 0 0; 0 cos(al) -sin(al); 0 sin(al) cos(al)];

    relT = cell(n_links,1);
    T = cell(n_links,1);
    z = cell(n_links,1);
    p = cell(n_links,1);
    euler = cell(n_links,1);
    for i = 1:n_links
        Tz = [Rz(dh(i,4)), [0;0;dh(i,3)]; [0 0 0 1]];
        Tx = [Rx(dh(i,2)), [dh(i,1);0;0]; [0 0 0 1]];
        relT{i} = Tz*Tx;
        if i == 1
            T{i} = base*relT{i};
        else
            T{i} = simplify(T{i-1}*relT{i});
        end
        z{i} = T{i}(1:3,3);
        p{i} = T{i}(1:3,4);
        if isequal(T{i}(1,3), sym('0'))
            euler{i} = [sym('0'); sym('0'); atan2(T{i}(2,1), T{i}(1,1))];
        else
            euler{i} = [atan2(T{i}(2,3), T{i}(1,3)); atan2(sqrt(T{i}(1,3)^2+T{i}(2,3)^2), T{i}(3,3)); atan2(T{i}(3,2), -T{i}(3,1))];
        end
    end
    robot.relT = relT;
    robot.T = T;
    robot.z = z;
    robot.p = p;
    robot.euler = euler;

    %% Jacobian

    prismatic_jacobian_column = @(z) [z; zeros(3,1)];
    rotoidal_jacobian_column = @(z, point, origin) [cross(z, point-origin); z];
    robot.point = sym('p', [3,1]);
    % first link
    if jointTypes(1) == 3 % prismatic
        jacobian = prismatic_jacobian_column(base(1:3,3));
    else
        jacobian = rotoidal_jacobian_column(base(1:3,3), point, base(1:3,4));
    end
    % other links
    for i = 2:n_links
        if jointTypes(i) == 3 % prismatic
            jacobian = [jacobian prismatic_jacobian_column(z{i-1})];
        else % rotoidal
            jacobian = [jacobian rotoidal_jacobian_column(z{i-1}, point, p{i-1})];
        end
    end
    robot.jacobian = jacobian;

    % end-effector, geometric
    J = subs(jacobian, point, p{end});
    J = simplify(J);
    robot.J = J;
    % end-effector, analytical
    % Ja = J; <- this shortcut can be used when manipulator is planar
    Ja = arrayfun(@(i) diff([p{end}; euler{end}], q(i)), 1:n_links, 'un', 0);
    Ja = cat(2, Ja{:});
    Ja = simplify(Ja);
    robot.Ja = Ja;

    % centers of mass, geometric
    Jp = cell(n_links,1);
    for i = 1:n_links
        pcom = T{i}*[coms{i}; 1];
        Jp{i} = subs(jacobian, point, pcom(1:3));
        Jp{i} = [Jp{i}(:,1:i) zeros(6,n_links-i)];
        Jp{i} = simplify(Jp{i});
    end
    robot.Jp = Jp;

    %% Inertia and other forces

    B = zeros(n_links);
    g = zeros(n_links,1);
    for i = 1:n_links
        Jpos = Jp{i}(1:3,:);
        Jor = Jp{i}(4:6,:);
        I = T{i}(1:3,1:3) * Is{i} * T{i}(1:3,1:3)';
        B = B + simplify(masses(i)*Jpos'*Jpos + Jor'*I*Jor);
        g = g - (masses(i)*g0'*Jpos)';
    end 
    B = simplify(B);
    robot.B = B;
    robot.g = g;
    
    Bder = arrayfun(@(i) diff(B, q(i)), 1:n_links, 'un', 0);
    Bder = cat(3, Bder{:});
    C = 0.5 * (Bder + permute(Bder,[1 3 2]) - permute(Bder,[3 1 2]));  % Bder{k}(i,j) + Bder{j}(i,k) - Bder{i}(j,k)
    % C = simplify(C); % may be avoided cause it's not involved in inverses
    robot.C = C;

    %% Inverses

    % Binv = inv(rewrite(B,'exp'));
    % Binv = simplify(rewrite(Binv,'exp')); % 40 min :O
    % robot.Binv = Binv;%matlabFunction(Binv);
    
    % assume(q, 'real');
    % Jpinv = pinv(J);
    % Jpinv = simplify(rewrite(Jpinv,'exp'));
    % robot.Jpinv = Jpinv
    
    % Ba = inv(Ja*Binv*Ja');
    
    %% Jacobian derivative
    
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

end

