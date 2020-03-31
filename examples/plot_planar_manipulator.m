function plot_planar_manipulator(q, robot)
% plot_planar_manipulator
%   q: [time, qi]
%   robot: structure containing manipulator data

    T = cellfun(@(exp) matlabFunction(exp), robot.T, 'un', 0);

    origin = robot.base*[0;0;0;1];
    o1 = arrayfun(@(q1) T{1}(q1)*origin, q(:,1),'un',0);
    o2 = arrayfun(@(q1,q2) T{2}(q1,q2)*origin, q(:,1),q(:,2),'un',0);
    o3 = arrayfun(@(q1,q2,q3) T{3}(q1,q2,q3)*origin, q(:,1),q(:,2),q(:,3),'un',0);
    o4 = arrayfun(@(q1,q2,q3,q4) T{4}(q1,q2,q3,q4)*origin, q(:,1),q(:,2),q(:,3),q(:,4),'un',0);

    o1 = reshape(cell2mat(o1), 4, []);
    o2 = reshape(cell2mat(o2), 4, []);
    o3 = reshape(cell2mat(o3), 4, []);
    o4 = reshape(cell2mat(o4), 4, []);

    o1 = o1(1:2,:);
    o2 = o2(1:2,:);
    o3 = o3(1:2,:);
    o4 = o4(1:2,:);

    %figure;
    plotx = [0];
    ploty = [0];
    h = plot(plotx,ploty);
    xlim([-0.5 0.5]);
    ylim([-0.5 0.5]);
    h.XDataSource = 'plotx';
    h.YDataSource = 'ploty';
    for i=1:size(q,1)
        plotx = [origin(1) o1(1,i) o2(1,i) o3(1,i) o4(1,i)];
        ploty = [origin(2) o1(2,i) o2(2,i) o3(2,i) o4(2,i)];
        refreshdata(h, 'caller');
        pause(0.001);
    end
    
end

