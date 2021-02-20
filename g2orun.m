%% Create g2o file from raw Victoria Park dataset
%% Ngo Thanh Tung, HUST

function g2orun()

global Param;
global States;
global Data;
global Threshold;
global G2O;

Data = load_vp_si();
Param = set_params();
Threshold = set_Threshold();

% global AAr;
% AAr = [0:360]*pi/360;

States.X_ab = zeros(3,1); % Robot position in absolute coordinates system
X_p = zeros(3,1); % Robot position prediction [x; y; phi]
M = zeros(3,1); % Global map [x_i; y_i; i]
X = zeros(3,1); % Robot's trajectory 3xn
L_s = zeros(3,1); % Laser position predicted 3xn
ci = 1; % control index
global t;
t = min(Data.Laser.time(1), Data.Control.time(1)); % time stamp
% States.X_ab(3) = 36*pi/180;

States.vertex = zeros(3,1);
G2O.SE2_vertex = zeros(4,1);
G2O.SE2_vertex(4) = 1;
G2O.edge = zeros(4,1);

global Z_full;
Z_full = zeros(3,1);
test_step = 500;

% for k=1:length(Data.Laser.time)
for k=1:test_step % Test algorithm
    while (Data.Control.time(ci)<Data.Laser.time(k))
        t = Data.Control.time(ci);
        dt = Data.Control.time(ci+1) - t;
%         t = Data.Control.time(ci);
        u = [Data.Control.ve(ci), Data.Control.alpha(ci)]';
        X_p = motion_model(u, dt);
        States.X_ab = States.X_ab + X_p;
        ci = ci+1;
        X = [X States.X_ab];
        
%         if (new_XY_vertex(States.X_ab))
%             add_XY_vertex(States.X_ab);
%         end
        if (new_SE2_vertex(States.X_ab))
            add_SE2_vertex(States.X_ab);
        end
        
    end
    
    dt = Data.Laser.time(k) - t;
    t = Data.Laser.time(k);
    if (ci == 1)
        u = [0 0]';
    else
        u = [Data.Control.ve(ci-1), Data.Control.alpha(ci-1)]';
    end
    
    X_p = motion_model(u, dt);
    z = detectTreesI16(Data.Laser.ranges(k,:));
   
    % detectTreesI16() return x, a 3xn matrix, where
    % x(1,i) distance to i-landmark center
    % x(2,i) angle of i-landmark center (in radians)
    % x(3,i) i-landmark diameter
    
    %% Convert relative coordinates to absolute coordinate
    if (length(z)==0) 
        continue;
    end
    [X_abs, z_ab] = rel2abs(X_p, z);
%     States.X_ab
    L_s = [L_s X_abs];
    Z_full = [Z_full z_ab];
    
    
end

X;
G2O.SE2_vertex;
test_gps_step = 4000;
figure;
gt_x = Data.Gps.x - Data.Gps.x(1);
gt_y = Data.Gps.y - Data.Gps.y(1);
plot(gt_x(1:test_gps_step), gt_y(1:test_gps_step), 'b-');
hold on;
plot (X(1,:), X(2,:), 'r+');
% hold on;
plot (L_s(1,:), L_s(2,:), 'g-');
% plot (Z_full(1,:), Z_full(2,:), 'c*');
plot (G2O.SE2_vertex(1,:), G2O.SE2_vertex(2,:), 'c*');
hold off;

write_g2o_file(G2O.SE2_vertex);

%==========================================================================
function [X_abs,z_abs] = rel2abs(X_p,z)

% X_p;
global Param;
global States;
% X_ab;
X_abs = States.X_ab + X_p;
phi = X_abs(3);

X_v = zeros(3,1);
X_v(1) = X_abs(1) + Param.a*cos(phi)-Param.b*sin(phi);
X_v(2) = X_abs(2) + Param.a*sin(phi)-Param.b*cos(phi);

length(z)

z_abs = zeros(3,length(z));
z_abs(1) = z(1).*cos(phi + z(2)) + X_v(1);
z_abs(2) = X_v(2) + z(1).*sin(phi + z(2));
z_abs(3) = z(3);
