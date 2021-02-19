%% Create g2o file from raw Victoria Park dataset
%% Ngo Thanh Tung, HUST

function g2orun()

%function runvp(nSteps,pauseLen, makeVideo)

global Param;
global States;
global Data;

Param.vp=1;
Param.sim=0;

% if ~exist('nSteps','var') || isempty(nSteps)
%     nSteps = inf;
% end

% if ~exist('pauseLen','var')
%     pauseLen = 0; % seconds
% end

Data = load_vp_si();

% draw_gt(Data);

% if makeVideo
%     try
%         votype = 'avifile';
%         vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
%     catch
%         votype = 'VideoWriter';
%         %%%vo = VideoWriter('video', 'MPEG-4');
%         %%%set(vo, 'FrameRate', min(5, 1/pauseLen));
%         %%%open(vo);
%     end
% end

% Initalize Params
%===================================================
% vehicle geometry
Param.a = 3.78; % [m]
Param.b = 0.50; % [m]
Param.L = 2.83; % [m]
Param.H = 0.76; % [m]

% 2x2 process noise on control input
sigma.vc = 0.02; % [m/s]
sigma.alpha = 2*pi/180; % [rad]
Param.Qu = diag([sigma.vc, sigma.alpha].^2);

% 3x3 process noise on model error
sigma.x = 0.1; % [m]
sigma.y = 0.1; % [m]
sigma.phi = 0.5*pi/180; % [rad]
Param.Qf = diag([sigma.x, sigma.y, sigma.phi].^2);

% 2x2 observation noise
sigma.r = 0.05; % [m]
sigma.beta = 1*pi/180; % [rad]
Param.R = diag([sigma.r, sigma.beta].^2);
%===================================================

% Initialize State
%===================================================
% State.Ekf.mu = [Data.Gps.x(2), Data.Gps.y(2), 36*pi/180]';
% State.Ekf.Sigma = zeros(3);
% State.Ekf.aug_mu = State.Ekf.mu;
% State.Ekf.Observed_landmarks = [];
% State.Ekf.data_assoc = [];
% State.Ekf.predMu = State.Ekf.mu;
% State.Ekf.predSigma = State.Ekf.Sigma;
% Param.updateMethod='seq'
% State.Ekf.run_time_pred = zeros(nSteps);
% State.Ekf.run_time_update = zeros(nSteps);
% State.Ekf.run_time_landmarks=zeros(nSteps);
global AAr;
AAr = [0:360]*pi/360;


% figure(1); clf;
% axis equal;

% global X_ab; % Robot position in absolute coordinates system
States.X_ab = zeros(3,1); % Robot position in absolute coordinates system
X_p = zeros(3,1); % Robot position prediction [x; y; phi]
M = zeros(3,1); % Global map [x_i; y_i; i]
X = zeros(3,1); % Robot's trajectory 3xn
L_s = zeros(3,1); % Laser position predicted 3xn
ci = 1; % control index
global t;
t = min(Data.Laser.time(1), Data.Control.time(1)); % time stamp
% States.X_ab(3) = 36*pi/180;

global Z_full;
Z_full = zeros(3,1);
test_step = 700;

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
%         X = [X X_p];
        X = [X States.X_ab];
        
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
    
    
%     if (~isempty(z))
%         z_2 = z;
%         z_2(2,:) = z_2(2,:) - pi/2;
%     end

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
test_gps_step = 4000;
figure;
gt_x = Data.Gps.x - Data.Gps.x(1);
gt_y = Data.Gps.y - Data.Gps.y(1);
plot(gt_x(1:test_gps_step), gt_y(1:test_gps_step), 'b-');
hold on;
plot (X(1,:), X(2,:), 'r+');
% hold on;
plot (L_s(1,:), L_s(2,:), 'g-');
plot (Z_full(1,:), Z_full(2,:), 'c*');
hold off;

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
