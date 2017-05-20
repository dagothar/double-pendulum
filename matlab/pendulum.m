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
U = -m1 * g * y1 - m2 * g * y2
L = T - U

%% Lagrange derivatives
syms q1 q2 dq1 dq2 d2q1 d2q2
dLddth1 = subs(diff(subs(L, dth1, dq1), dq1), dq1, dth1);
dLddth1dt = diff(dLddth1, t);
dLdth1 = diff(subs(L, th1, q1), q1);
dLddth2 = subs(diff(subs(L, dth2, dq2), dq2), dq2, dth2);
dLddth2dt = diff(dLddth2, t);
dLdth2 = diff(subs(L, th2, q2), q2);
e1 = dLddth1dt - dLdth1;
e2 = dLddth2dt - dLdth2;
e1 = subs(e1, {th1, dth1, d2th1, th2, dth2, d2th2}, {q1, dq1, d2q1, q2, dq2, d2q2});
e2 = subs(e2, {th1, dth1, d2th1, th2, dth2, d2th2}, {q1, dq1, d2q1, q2, dq2, d2q2});

%% Euler-Lagrange equations
eqns = [
    e1 == tau1
    e2 == tau2
    ]

%% Solve equations
[d2q1, d2q2] = solve(eqns, [d2q1 d2q2]);
d2q1 = simplify(d2q1)
d2q2 = simplify(d2q2)

%% Save to file
fid = fopen('equations.txt', 'wt');
T = subs(T, {th1, dth1, th2, dth2}, {q1 dq1, q2, dq2});
U = subs(U, {th1, dth1, th2, dth2}, {q1 dq1, q2, dq2});
fprintf(fid, 'd2q1 = %s\n', char(d2q1));
fprintf(fid, 'd2q2 = %s\n', char(d2q2));
fprintf(fid, 'T = %s\n', char(T));
fprintf(fid, 'U = %s\n', char(U));
fclose(fid);
