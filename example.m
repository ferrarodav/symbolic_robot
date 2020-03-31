robot_name = 'ideal'
perturbationRelativeError = 0 % 0.1

%% Robot Definition
DHParams = [0.12 0 0 0; % the i-th row has the DH parameters of the i-th link
            0.134 0 0 0;
            0.134 0 0 0; 
            0.037 0 0 0];
jointTypes = 'rrrr'; % r for rotational link, p for prismatic
coms = {[-0.06; 0; 0], % coordinates of the i-th center of mass, referred to the i-th ref system (at the end of the link)
        [-0.067; 0; 0], 
        [-0.067; 0; 0], 
        [-0.0185; 0; 0]}; 
masses = [30.67, 131.26, 131.26, 300]./1000; 
Is = {[18435.99, -52.98, 155.96; -52.98, 175242.33, 0.00; 155.96, 0.00, 159813.64]/1e9, % Inertia matrix of each link, referred to its center of mass 
      [27254.18, -124.88, 11769.80; -124.88, 881064.41, -36.36; 11769.80, -36.36, 865760.77]/1e9,
      [27254.18, -124.88, 11769.80; -124.88, 881064.41, -36.36; 11769.80, -36.36, 865760.77]/1e9,
      [5595.17, -408.61, 0.11; -408.61, 8440.00, 0.00; 0.11, 0.00, 3458.06]/1e9};
g0 = [0; -9.81; 0]; % gravity vector
base = eye(4); % base reference frame transformation matrix

%% Inject noise into the the robot parameters
if perturbationRelativeError > 0
    perturbate = @(var) var.*(1 + perturbationRelativeError.*(-1 + 2*rand(size(var))));

    DHParams = perturbate(DHParams);
    coms = cellfun(perturbate, coms, 'UniformOutput', false);
    masses = perturbate(masses);
    Is = cellfun(perturbate, Is, 'UniformOutput', false);
end 

%% Generate symbolic functions and save them
robot = getRobot(DHParams, jointTypes, coms, masses, Is, g0, base);
save('robot.mat', 'robot');

%% Generate matlab code to compute the symbolically obtained functions
matlabFunction(robot.J, 'File', [robot_name '_Jg'], 'Optimize', true, 'Vars', {robot.q}, 'Outputs', {'Jg'});
matlabFunction(robot.B, 'File', [robot_name '_B'], 'Optimize', true, 'Vars', {robot.q}, 'Outputs', {'B'});
matlabFunction(robot.C, 'File', [robot_name '_C'], 'Optimize', true, 'Vars', {robot.q}, 'Outputs', {'C'});
matlabFunction(robot.g, 'File', [robot_name '_G'], 'Optimize', true, 'Vars', {robot.q}, 'Outputs', {'g'});
matlabFunction(robot.Ja_dot, 'File', [robot_name '_Ja_dot'], 'Optimize', true, 'Vars', {robot.q, robot.q_dot}, 'Outputs', {'Ja_dot'});
cellfun(@(expr,i) ...
    matlabFunction(expr, 'File', [robot_name '_T' num2str(i)], 'Optimize', true, 'Vars', {robot.q}, 'Outputs', {['T' num2str(i)]}), ...
    robot.T, num2cell([1:length(robot.T)]'), 'un', 0);
matlabFunction([robot.p{end}; robot.euler{end}], 'File', [robot_name '_EEpose'], 'Optimize', true, 'Vars', {robot.q}, 'Outputs', {'x'});


%% Plot some robot movement
% a simple movement between two random configurations
test_qs = arrayfun(@(from,to) linspace(from, to, 500), 
                   2*pi*rand(size(DHParams,1),1), 
                   2*pi*rand(size(DHParams,1),1), 
                   'un', 0);
figure;
plotPlanarRobot(cell2mat(test_qs)', robot);