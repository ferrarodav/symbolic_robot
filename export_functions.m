function export_functions(robot, prefix, optimize)
%EXPORT FUNCTIONS Generate matlab code to compute the manipulator symbolical functions

    % Trasformation matrices
    cellfun(@(expr,i) ...
        matlabFunction(expr, 'File', [prefix '_T' num2str(i)], 'Optimize', optimize, 'Vars', {robot.q}, 'Outputs', {['T' num2str(i)]}), ...
        robot.T, num2cell([1:length(robot.T)]'), 'un', 0);
    % End effector pose
    matlabFunction([robot.p{end}; robot.euler{end}], 'File', [prefix '_EEpose'], 'Optimize', optimize, 'Vars', {robot.q}, 'Outputs', {'x'});
    % Jacobian
    matlabFunction(robot.J, 'File', [prefix '_Jg'], 'Optimize', optimize, 'Vars', {robot.q}, 'Outputs', {'Jg'});
    % matlabFunction(robot.Ja, 'File', [prefix '_Ja'], 'Optimize', optimize, 'Vars', {robot.q}, 'Outputs', {'Ja'});
    
    % Dynamics
    if isfield(robot, 'B')
        matlabFunction(robot.B, 'File', [prefix '_B'], 'Optimize', optimize, 'Vars', {robot.q}, 'Outputs', {'B'});
        matlabFunction(robot.C, 'File', [prefix '_C'], 'Optimize', optimize, 'Vars', {robot.q}, 'Outputs', {'C'});
        matlabFunction(robot.g, 'File', [prefix '_G'], 'Optimize', optimize, 'Vars', {robot.q}, 'Outputs', {'g'});
        % matlabFunction(robot.Ja_dot, 'File', [prefix '_Ja_dot'], 'Optimize', optimize, 'Vars', {robot.q, robot.q_dot}, 'Outputs', {'Ja_dot'});  
    end

end