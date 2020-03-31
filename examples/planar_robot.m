%% Robot Definition
% Kinematics
DHParams = [0.12 0 0 0; % the i-th row has the DH parameters of the i-th link
            0.134 0 0 0;
            0.134 0 0 0; 
            0.037 0 0 0];
jointTypes = 'rrrr'; % r for rotational link, p for prismatic
base = eye(4); % base reference frame transformation matrix
% Dynamics
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

%%
perturbationRelativeError = 0 % 0.1
%% Eventually inject noise into the the robot parameters
if perturbationRelativeError > 0
    perturbate = @(var) var.*(1 + perturbationRelativeError.*(-1 + 2*rand(size(var))));

    DHParams = perturbate(DHParams);
    coms = cellfun(perturbate, coms, 'UniformOutput', false);
    masses = perturbate(masses);
    Is = cellfun(perturbate, Is, 'UniformOutput', false);
end 

%% Generate symbolic functions, save them into a mat file and export them to optimized matlab code
robot = get_manipulator(DHParams, jointTypes, base);
robot = add_manipulator_dynamics(robot, coms, masses, Is, g0);
save('robot.mat', 'robot');
export_functions(robot, 'test_robot', true)

%% Plot some robot movement
% a simple movement between two random configurations
test_qs = arrayfun(@(from,to) linspace(from, to, 500), 
                   2*pi*rand(size(DHParams,1),1), 
                   2*pi*rand(size(DHParams,1),1), 
                   'un', 0);
figure;
plot_planar_robot(cell2mat(test_qs)', robot);