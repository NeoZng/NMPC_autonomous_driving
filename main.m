clc; 
clear; 
close all;
import casadi.*

%% Generate road lane and obstacles
N = 20;
N_c = 1;
[Ref,End,obs,r]=loadMap(N);
%% CasADi objec  t,construct NLP problem
% State and input dimensions
nx = 6; nu = 2;
% Create an CasADi solver api
opt = casadi.Opti();
x = opt.variable(nx, N+1);
u = opt.variable(nu, N);
xt = opt.parameter(nx,1);
ref = opt.parameter(2, N+1);
cost = 0;
%      px, py, phi, vx, vy, w
Q = diag([1, 1]);   % State tracking cost
Q_t = diag([1, 1, 0, 0, 0, 0]); % Terminal state cost
%          a , delta
R = diag([0.1, 0.5]);         % Control effort cost
R_rate = diag([0.1, 0.1]);  % Control rate cost

% Constraints on input
a_max = 1.5;
a_min = -3;
d_max = 0.6;
d_min = -0.6;

% Cost function and constraints
opt.subject_to( x(:,1) == xt ); % initial state constraint
for k=1:N
  % dynamics
  opt.subject_to( x(:,k+1) == car.car_dynamics(x(:,k),u(:,k)) );
  % obstacle avoidance
  for j = 1:length(obs)
    opt.subject_to(norm(x(1:2,k) - obs(:,j), 2) >= (car.width + r));
  end
  % input constraints
  opt.subject_to( u(2,k) <= d_max );
  opt.subject_to( u(2,k) >= d_min );
  opt.subject_to( u(1,k) <= a_max );
  opt.subject_to( u(1,k) >= a_min );
  % tracking cost & control effort
  cost = cost + (x(1:2,k)-ref(:,k)).'*Q*(x(1:2,k)-ref(:,k))+u(:,k).'*R*u(:,k); 
  % penalize rate
  if k > 1
    cost = cost + (u(:,k)-u(:,k-1)).'*R_rate*(u(:,k)-u(:,k-1));
  end
end
 % terminal cost
cost = cost + (x(1:2,N+1)-ref(:,N+1)).'*(x(1:2,N+1)-ref(:,N+1));
opt.minimize(cost);

% suppress output
opts1 = struct('verbose',false);
opts1.print_time = false;
opts2 = struct('print_level',0);
opt.solver('ipopt',opts1,opts2);

%% System simulation

% MPC Initialization
x0 = [Ref(1,1); Ref(2,1); 0; 0; 0; 0];
% align initial orientation with the first line segment
x0(3) = atan2( (Ref(2,2)-Ref(2,1)) , (Ref(1,2)-Ref(1,1)));
X = [x0,zeros(nx,End)]; U = zeros(nu,End);
XPred = {};  UPred = {};

for k = 1:End-1
  % Initial state and reference points
  opt.set_value( xt, X(:, k) )
  opt.set_value( ref, Ref(:, k:k+N))
  % Solve NLP
  sol = opt.solve();
  % optimized sequences N
  XPred{k} = sol.value( x );
  UPred{k} = sol.value( u );
  % apply first control
  U(:,k) = UPred{k}(:,1);
  X(:,k+1) = car.car_dynamics( X(:,k), U(:,k) );
  
  opt.set_initial( x, [XPred{k}(:,2:end), car.car_dynamics(XPred{k}(:,end),[0;0])] )
  opt.set_initial( u, [UPred{k}(:,2:end), [0;0] ] )

  % update visualisation
  figure(1)
  hold on
  pred(k+1) = plot(XPred{k}(1,:), XPred{k}(2,:), 'g-', 'linewidth',1.5);
  car.vis_car(X(:,k))
  hold on
  delete(pred(k))
  pause(0.01)
end

%% save each frame as an image then create a video
% for i = 1:End
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     if i == 1
%         imwrite(imind,cm,'test.gif','gif', 'Loopcount',inf);
%     else
%         imwrite(imind,cm,'test.gif','gif','WriteMode','append');
%     end
% end

