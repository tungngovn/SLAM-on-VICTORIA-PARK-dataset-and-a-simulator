function draw_gt(Data)

gt_x = Data.Gps.x - Data.Gps.x(1);
gt_y = Data.Gps.y - Data.Gps.y(1);
plot(gt_x, gt_y, 'b.');
hold on
% plotbot(State.Ekf.mu(1), State.Ekf.mu(2), State.Ekf.mu(3), 'black', 1, 'blue', 1);
% hold on;
end