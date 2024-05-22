classdef car
  
  properties (Constant)
    width = 1.5;
    length = 3.2;
  end
  
  methods (Static)
    function x_pred = car_dynamics(x, u)
      % Parameters
      Ts = 0.1;  
      kf = -128916;  
      kr = -85944;   
      lf = 1.06;     
      lr = 1.85;     
      m = 1412;      
      Iz = 1536.7;
      Lk = lf*kf - lr*kr;
      
      x_pred = ...
       [x(1) + Ts * (x(4) * cos(x(3)) - x(5) * sin(x(3)));
        x(2) + Ts * (x(5) * cos(x(3)) + x(4) * sin(x(3)));
        x(3) + Ts * x(6);
        x(4) + Ts * u(1);
        (m * x(4) * x(5) + Ts * Lk * x(6) - Ts * kf * u(2) * x(4) - Ts * m * x(4)^2 * x(6)) / (m * x(4) - Ts * (kf + kr));
        (Iz * x(4) * x(6) + Ts * Lk * x(5) - Ts * lf * kf * u(2) * x(4)) / (Iz * x(4) - Ts * (lf^2 * kf + lr^2 * kr))];
    
    end

    function [] = vis_car(X)
      %Plots the car with wheels
          persistent car_G wheels_G
          w = car.width / 2;
          Lf = 1.06; 
          ext = 0.3;
          Lr = 1.85; 
          lf = Lf+ext;
          lr = Lr+ext;
          x = X(1);
          y = X(2);
          yaw = X(3); % vehicle orientation
      
          fl = [x;y] + [cos(yaw)*lf;sin(yaw)*lf] + [sin(yaw)*w;-cos(yaw)*w];
          fr = [x;y] + [cos(yaw)*lf;sin(yaw)*lf] - [sin(yaw)*w;-cos(yaw)*w];
          bl = [x;y] - [cos(yaw)*lr;sin(yaw)*lr] + [sin(yaw)*w;-cos(yaw)*w];
          br = [x;y] - [cos(yaw)*lr;sin(yaw)*lr] - [sin(yaw)*w;-cos(yaw)*w];

          % update car position
          delete(car_G)
          car_G = plot([fl(1),fr(1),br(1),bl(1),fl(1)],[fl(2),fr(2),br(2),bl(2),fl(2)],'Color', [0 0.4 0.7],'LineWidth',2.5);
          % legend for car
        end
      
  end
  
end