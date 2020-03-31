function [density] = plot_planar_workspace(robot, jointLimits, montecarloSimulations, gridPrecision)

    % The first part is ok for 3D manipulators also
    uniform = @(a,b,rows,cols) (b-a).*rand(rows,cols) + a;
    q = uniform(jointLimits(:,1), jointLimits(:,2), size(jointLimits,1), montecarloSimulations);
    x = zeros(montecarloSimulations, 6);
    
    dk = matlabFunction([robot.p{end}; robot.euler{end}]);
    q = num2cell(q);
    for i = 1:montecarloSimulations
        x(i,:) = dk(q{:,i});
    end
    
    % PLANAR so we take out z and rotations around x and y
    x = x(:, [1 2 6]); 
    
    position_in_grid = round(x./reshape(gridPrecision,1,[]));
    min_position = min(position_in_grid, [], 1);
    position_in_matrix = position_in_grid - min_position + 1;
    density = accumarray(position_in_matrix, 1);
    
    figure; title('Plane density (invariant to orientation)');
    max_position = max(position_in_grid, [], 1);
    x_pos = [min_position(1) max_position(1)].*gridPrecision(1)*100; % in cm
    y_pos = [min_position(2) max_position(2)].*gridPrecision(2)*100; % in cm
    plane_density = sum(density,3);
    image(x_pos, y_pos, plane_density'./max(plane_density(:)), 'CDataMapping', 'scaled');
    caxis([0 1]); colorbar;
    
%     figure; title('asd');
%     scatter3(100*x(:,1), 100*x(:,2), x(:,3),0.1,'filled');
    
    figure;
    psi_grid_size = size(density,3);
    psi_pos = linspace(min_position(3), max_position(3), 10).*gridPrecision(3);
    plane_density = cell(9,1);
    for i = 1:9
        it_start = ceil((i-1)*(psi_grid_size/9)) + 1;
        it_end = ceil(i*(psi_grid_size/9));
        plane_density{i} = sum(density(:,:,it_start:it_end), 3);
    end
    max_density = max(cellfun(@(c)max(c(:)), plane_density));
    for i = 1:9
        subplot(3, 3, i); 
        t = ['Psi ' num2str(psi_pos(i)) ' to ' num2str(psi_pos(i+1))];
        image(x_pos, y_pos, plane_density{i}'./max_density, 'CDataMapping', 'scaled');
        caxis([0 1]); colorbar;
        title(t);
    end
    
end