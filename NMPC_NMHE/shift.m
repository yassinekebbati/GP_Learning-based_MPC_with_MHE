%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/05/2022
% Control NMMPC-NMHE-Racing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x0, u0, z] = shift2(T, x0, u,vehicle_states)


[T,x] = ode45(@(t,x) fullmodel(t,x, u(1,:)'), T, vehicle_states);  

% vehicle_states  = [vx; vy; omega; 0; 0; 0]; %Simulator state vector    
z = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)]; % [x y phi vx vy w]
    
x0 = [x(end,1); x(end,2); x(end,3); sqrt(x(end,4)^2+x(end,5)^2)]; % [x y phi v]
% x0 = [x(end,1); x(end,2); x(end,3); x(end,4)]; % [x y phi v]


% st = x0;
% con = u(1,:)';
% f_value = f(st,con);
% st = st+ (T*f_value);
% x0 = full(st);

% t0 = t0 + T(1);
u0 = [u(2:size(u,1),:);u(size(u,1),:)];
end