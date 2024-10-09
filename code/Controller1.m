function func = Controller
% INTERFACE
%
%   sensors
%       .e_lateral      (error in lateral position relative to road)
%       .e_heading      (error in heading relative to road)
%       .wR             (angular velocity of right wheel)
%       .wL             (angular velocity of left wheel)
%       .r_road         (signed radius of curvature of road - to find the
%                        corresponding turning rate for a given forward
%                        speed: w_road = v_road/sensors.r_road)
%
%   references
%       (none)
%
%   parameters
%       .tStep      (time step)
%       .tauMax     (maximum wheel torque)
%       .roadwidth  (width of road - half on each side of centerline)
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%       .b          (distance between the two wheels)
%       .r          (radius of each wheel)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .tauR       (right wheel torque)
%       .tauL       (left wheel torque)

% Do not modify this part of the function.
func.init = @initControlSystem;
func.run = @runControlSystem;

% If you want sensor data to be delayed, set func.iDelay to some
% non-negative integer (this is the number of time steps of delay).
func.iDelay = 0;

end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [data] = initControlSystem(parameters,data)
load('DesignProblem04_EOMs.mat');
f = symEOM.f;
%defining variables
syms phi phidot v w tauR tauL elateral eheading vroad wroad rroad real;
b = 0.4;
R = 0.2;

phiddot = f(1);
vdot = f(2);
wdot = f(3);
elateraldot = -v*sin(eheading);
Eheadingdot = w - (v*cos(eheading)/(vroad + wroad*elateral))*wroad;
eheadingdot = simplify(subs(Eheadingdot, vroad, (wroad*rroad)));
wR = (1/R)*(v+w*(b/2));
wL = (1/R)*(v-w*(b/2));

%defining equilibriums
data.var = [phi, phidot, v, w, tauR, tauL, elateral, eheading, rroad];
data.vareq = [0, 0, 6, 0, 0, 0, 0, 0, 10^150];

s = [phi, phidot, v, w, elateral, eheading]; 
u = [tauR, tauL];
sd = [phidot; phiddot; vdot; wdot; elateraldot; eheadingdot];
y = [wR; wL; elateral; eheading];


A = jacobian(sd,s);
B = jacobian(sd,u);
C = jacobian(y,s);
D = jacobian(y,u);

data.A = double(subs(A, {phi, phidot, v, w, tauR, tauL, elateral, eheading, rroad}, {data.vareq}));
data.B = double(subs(B, {phi, phidot, v, w, tauR, tauL, elateral, eheading, rroad}, {data.vareq}));
data.C = double(subs(C, {phi, phidot, v, w, tauR, tauL, elateral, eheading, rroad}, {data.vareq}));
data.D = double(subs(D, {phi, phidot, v, w, tauR, tauL, elateral, eheading, rroad}, {data.vareq}));

data.Qc = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 450000 0; 0 0 0 0 0 450000];
data.Rc = [10];
data.K = lqr(data.A,data.B,data.Qc,data.Rc);

data.Qo = [30];
data.Ro = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 5 0; 0 0 0 0 0 5];
data.L = lqr(data.A',data.C',inv(data.Ro),inv(data.Qo))';

data.xhat = [-data.vareq(1); -data.vareq(2); -data.vareq(3); -data.vareq(4); -data.vareq(7); -data.vareq(8)];
data.ysol = [(1/parameters.r)*(data.vareq(3)+data.vareq(4)*(parameters.b/2)); (1/parameters.r)*(data.vareq(3)-data.vareq(4)*(parameters.b/2)); data.vareq(7); data.vareq(8)];

save initVars.mat

end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)
u = -data.K*data.xhat;
data.y = [sensors.wR; sensors.wL; sensors.e_lateral; sensors.e_heading] - data.ysol;
data.xhat = data.xhat + parameters.tStep*(data.A*data.xhat + data.B*u - data.L*(data.C*data.xhat-data.y));

actuators.tauR = u(1) + data.vareq(5);
actuators.tauL = u(2) + data.vareq(6);
end