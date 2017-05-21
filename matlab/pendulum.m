clc; clear all; close all;

%% Symbolic variables
syms t x1 y1 x2 y2 m1 m2 l1 l2 g tau1 tau2 th1(t) th2(t)

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

T = 0.5 * m1 * v1sq + 0.5 * m2 * v2sq
V = -m1 * g * y1 - m2 * g * y2
L = T - V

%% Lagrange derivatives
syms q1 q2 dq1 dq2 d2q1 d2q2 

d_L_dq1 = diff(subs(L, dth1, dq1), dq1);
d_L_dq2 = diff(subs(L, dth2, dq2), dq2);
d_L_q1 = diff(subs(L, th1, q1), q1);
d_L_q2 = diff(subs(L, th2, q2), q2);

d_L_dq1 = subs(d_L_dq1, dq1, dth1);
d_L_dq2 = subs(d_L_dq2, dq2, dth2);
d_L_q1 = subs(d_L_q1, q1, th1);
d_L_q2 = subs(d_L_q2, q2, th2);

d_L_t1 = diff(d_L_dq1, t);
d_L_t2 = diff(d_L_dq2, t);

e1 = d_L_t1 + d_L_q1;
e2 = d_L_t2 + d_L_q2;

T = subs(T, {th1, dth1, d2th1, th2, dth2, d2th2}, {q1, dq1, d2q1, q2, dq2, d2q2});
V = subs(V, {th1, dth1, d2th1, th2, dth2, d2th2}, {q1, dq1, d2q1, q2, dq2, d2q2});
e1 = subs(e1, {th1, dth1, d2th1, th2, dth2, d2th2}, {q1, dq1, d2q1, q2, dq2, d2q2});
e2 = subs(e2, {th1, dth1, d2th1, th2, dth2, d2th2}, {q1, dq1, d2q1, q2, dq2, d2q2});

%% Euler-Lagrange equations
eqns = [
    e1 == 0
    e2 == 0
    ]

%% Solve equations
[d2q1, d2q2] = solve(eqns, [d2q1 d2q2]);
T = simplify(T)
V = simplify(V)
d2q1 = simplify(d2q1)
d2q2 = simplify(d2q2)

%% Save to file
fid = fopen('equations.txt', 'wt');
fprintf(fid, 'T = %s\n', char(T));
fprintf(fid, 'V = %s\n', char(V));
fprintf(fid, 'd2q1 = %s\n', char(d2q1));
fprintf(fid, 'd2q2 = %s\n', char(d2q2));
fclose(fid);
