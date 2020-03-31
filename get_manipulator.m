function [robot] = get_manipulator(DHParams, jointTypes, base)
%GET MANIPULATOR Symbolically computes the expressions of kinematics quantities relative to the specified manipulator.

    %% Input parsing

    robot = struct();
    robot.fixedDHParams = DHParams;
    robot.jointTypes = jointTypes;
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

    % End-effector jacobian, geometric
    J = subs(jacobian, point, p{end});
    J = simplify(J);
    robot.J = J;
    % End-effector jacobian, analytical
    % Ja = J; <- this shortcut can be used when manipulator is planar
    Ja = arrayfun(@(i) diff([p{end}; euler{end}], q(i)), 1:n_links, 'un', 0);
    Ja = cat(2, Ja{:});
    Ja = simplify(Ja);
    robot.Ja = Ja;

    % Geometric jacobian inverse
    % assume(q, 'real');
    % Jpinv = pinv(J);
    % Jpinv = simplify(rewrite(Jpinv,'exp'));
    % robot.Jpinv = Jpinv

end

