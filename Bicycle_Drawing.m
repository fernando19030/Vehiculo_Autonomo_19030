% figure(1);
% fill([-0.2, 0.2, 0.2, -0.2], [-0.1, -0.1, 0.1, 0.1], 'k');
% hold on;
% fill([-0.8, -0.2, -0.2, -0.8], [-0.05, -0.05, 0.05, 0.05], 'w');
% hold on;
% fill([-1.2, -0.8, -0.8, -1.2], [-0.1, -0.1, 0.1, 0.1], 'k');
% hold on;
% axis([-3 3 -3 3])
% grid on;

x = -10;
y = -5;

BV_front = [-0.2, 0.2, 0.2, -0.2; -0.1, -0.1, 0.1, 0.1];
IV_front = [cos(theta), -sin(theta); sin(theta), cos(theta)] * BV_front;
bodyplot_front = fill(IV_front(1,:) + x, IV_front(2,:) + y, [0.5,0.5,0.5]);
BV_center = [-0.8, -0.2, -0.2, -0.8; -0.05, -0.05, 0.05, 0.05];
IV_center = [cos(theta), -sin(theta); sin(theta), cos(theta)] * BV_center;
bodyplot_center = fill(IV_center(1,:) + x, IV_center(2,:) + y, [0.5,0.5,0.5]);
BV_back = [-1.2, -0.8, -0.8, -1.2; -0.1, -0.1, 0.1, 0.1];
IV_back = [cos(theta), -sin(theta); sin(theta), cos(theta)] * BV_back;
bodyplot_back = fill(IV_back(1,:) + x, IV_back(2,:) + y, [0.5,0.5,0.5]);

bodyplot_back.XData = IV_back(1,:) + x;
bodyplot_back.YData = IV_back(2,:) + y;

axis([-15 15 -15 15])