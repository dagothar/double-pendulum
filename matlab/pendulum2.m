clc; clear all; close all;

%% Symbolic variables
syms t m1 m2 l1 l2 g tau1 tau2 th1(t) th2(t)

dth1 = diff(th1, t);
dth2 = diff(th2, t);
d2th1 = diff(th1, t, t);
d2th2 = diff(th2, t, t);

%% Pendulum positions
x1 = l1 * sin(th1)
y1 = l1 * cos(th1)
x2 = x1 + l2 * sin(th2)
y2 = y1 + l2 * cos(th2)

%% Velocities
vx1 = diff(x1, t);
vy1 = diff(y1, t);
vx2 = diff(x2, t);
vy2 = diff(y2, t);

%% Lagrangian
v1sq = vx1^2 + vy1^2;
v2sq = vx2^2 + vy2^2;

T = simplify(0.5 * m1 * v1sq + 0.5 * m2 * v2sq)
V = simplify(-m1 * g * y1 - m2 * g * y2)
L = simplify(T - V)

%% Lagrange derivatives
syms q1 dq1 d2q1 q2 dq2 d2q2

% Manually substitute...
L = (l1^2*m1*dq1^2)/2 + (l1^2*m2*dq1^2)/2 + (l2^2*m2*dq2^2)/2 + g*m2*(l1*cos(q1)) + l2*cos(q2) + g*l1*m1*cos(q1) + l1*l2*m2*cos(q1 - q2)*dq1*dq2;

d_L_dq1 = diff(L, dq1)
d_L_q1 = diff(L, q1)

d_L_dq2 = diff(L, dq2)
d_L_q2 = diff(L, q2)

syms Q1(t) Q2(t) dQ1(t) dQ2(t)

% Manually substitute...
d_L_dq1 = dQ1(t)*l1^2*m1 + dQ1(t)*l1^2*m2 + dQ2(t)*l1*l2*m2*cos(Q1(t) - Q2(t));
d_L_dq2 = dQ2(t)*m2*l2^2 + dQ1(t)*l1*m2*cos(Q1(t) - Q2(t))*l2

d_L_t1 = diff(d_L_dq1, t)
d_L_t2 = diff(d_L_dq2, t)


% Manually substitute...
d_L_t1 = l1^2*m1*d2q1 + l1^2*m2*d2q1 + l1*l2*m2*cos(q1 - q2)*d2q2 - l1*l2*m2*sin(q1 - q2)*dq2*(dq1 - dq2)
d_L_t2 = l2^2*m2*d2q2 + l1*l2*m2*cos(q1 - q2)*d2q1 - l1*l2*m2*sin(q1 - q2)*dq1*(dq1 - dq2);

%% Euler-Lagrange equations
e1 = d_L_t1 - d_L_q1
e2 = d_L_t2 - d_L_q2

eqns = [
    e1 == 0
    e2 == 0
    ]

%% Solve equations
[d2q1, d2q2] = solve(eqns, [d2q1 d2q2]);
d2q1 = simplify(d2q1)
d2q2 = simplify(d2q2)

%% Save to file
fid = fopen('equations.txt', 'wt');
fprintf(fid, 'T = %s\n', char(T));
fprintf(fid, 'V = %s\n', char(V));
fprintf(fid, 'd2q1 = %s\n', char(d2q1));
fprintf(fid, 'd2q2 = %s\n', char(d2q2));
fclose(fid);