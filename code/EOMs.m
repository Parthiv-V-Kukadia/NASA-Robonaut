load('DesignProblem04_EOMs.mat')
f = symEOM.f;

%define 
syms phi phidot v w tauR tauL real
syms e_lateral e_heading v_road w_road r_road real
b = 0.4;
R = 0.2;

phiddot = f(1);
vdot = f(2);
wdot = f(3);
e_lateraldot = -v*sin(e_heading);
e_headingdot=w-((v*cos(e_heading)/(v_road+w_road*e_lateral)))*w_road;
e_headingdot = simplify(subs(e_headingdot, v_road, (w_road*r_road)));
wR = (1/R)*(v+w*(b/2));
wL = (1/R)*(v-w*(b/2));

%Find Equilibrium
var = [phi, phidot, v, w, tauR, tauL, e_lateral, e_heading, r_road];
var_guess = [0, 0, 6, 0, 0, 0, 0, 0, 10^(150)];

%Linearizing the system
x = [phi, phidot, v, w, e_lateral, e_heading];
u = [tauR, tauL];
xdot = [phidot; phiddot; vdot; wdot; e_lateraldot; e_headingdot];
y_output = [wR; wL; e_lateral; e_heading];

A_sym = jacobian(xdot,x);
B_sym = jacobian(xdot,u);
C_sym = jacobian(y_output,x);

A = double(subs(A_sym, {phi, phidot, v, w, tauR, tauL, e_lateral, e_heading, r_road}, {var_guess}))
B = double(subs(B_sym, {phi, phidot, v, w, tauR, tauL, e_lateral, e_heading, r_road}, {var_guess}))
C = double(subs(C_sym, {phi, phidot, v, w, tauR, tauL, e_lateral, e_heading, r_road}, {var_guess}))


%Checking for controlability  --> 1 = yes
W = ctrb(A,B);
iscont = logical(length(A) == rank(W));
rank(W)

%Checking A for stability - if all negative real parts
stabA = eig(A)

%Checking for observability --> 1  = yes
Obs = obsv(A,C);
isobs = logical(length(A) == rank(Obs))

%Defining state cost weighted matrix (Qc) and input cost weighted matrix
%(Rc)
Qc = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 450000 0; 0 0 0 0 0 450000];
Rc = [10];


%Define observer matrices (Qo, Ro)
Qo = [30];
Ro = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 5 0; 0 0 0 0 0 5];

%Define K and L matrices
K = lqr(A,B,Qc,Rc)
L = lqr(A',C',inv(Ro),inv(Qo))'

%Check A-BK and A-LC stability
stabABK = eig(A - B*K)
stabALC = eig(A - L*C)

save initVars.mat

