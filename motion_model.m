function X_p = motion_model(u, dt)
% Motion model for Victoria Park dataset
global Param;
%global State;
global States;

X_p = zeros(3,1);
%F = [eye(3), zeros(3,size(State.Ekf.mu,1)-3)];
ve = u(1,1);
alpha = u(2,1);
vc = ve/(1-tan(alpha)*(Param.H/Param.L)); %% CALCULATE VC
%phi = State.Ekf.mu(3,1);
% phi = 36*pi/180;
phi = States.X_ab(3);
% X_p(1,1) = States.X_ab(1) + dt*(vc*cos(phi)-(vc/Param.L)*tan(alpha)*(Param.a*sin(phi)+Param.b*cos(phi)))+ Param.Qu(1,1); %%CHANGE IN X
% X_p(2,1) = States.X_ab(2) + dt*(vc*sin(phi)+(vc/Param.L)*tan(alpha)*(Param.a*cos(phi)-Param.b*sin(phi)))+ Param.Qu(1,1); %% CHANGE IN Y
% X_p(3,1) = States.X_ab(3) + dt*(vc/Param.L)*tan(alpha) + Param.Qu(2,2); %CHANGE IN THETA
X_p(1,1) = dt*(vc*cos(phi)-(vc/Param.L)*tan(alpha)*(Param.a*sin(phi)+Param.b*cos(phi)))+ Param.Qu(1,1); %%CHANGE IN X
X_p(2,1) = dt*(vc*sin(phi)+(vc/Param.L)*tan(alpha)*(Param.a*cos(phi)-Param.b*sin(phi)))+ Param.Qu(1,1); %% CHANGE IN Y
X_p(3,1) = dt*(vc/Param.L)*tan(alpha) + Param.Qu(2,2); %CHANGE IN THETA
%State.Ekf.predMu = State.Ekf.mu + F'*motion_model; %%%UPDATE MU ACCORDING TO THE GIVEN MOTION MODEL
%State.Ekf.predMu(3,1) = minimizedAngle(State.Ekf.predMu(3,1));
%G = eye(size(F,2)) + F'* [0,0, -dt*(vc*sin(phi)+(vc/Param.L)*tan(alpha)*(Param.a*cos(phi)-Param.b*sin(phi)));... %deriv. of process model
%                          0,0, dt*(vc*cos(phi)-(vc/Param.L)*tan(alpha)*(Param.a*sin(phi)+Param.b*cos(phi)));...
%                          0,0,0;]*F;             
%State.Ekf.predSigma = G * State.Ekf.Sigma * G' + F' * Param.Qf * F ;  %%UPDATE SIGMA ACCORDING TO THE MOTION MODEL
end
