dt = 1; % timestep (1s)
g = 9.81; % gravity (m/s^2)

A = [1, 0, dt, 0;
    0,  1, 0,  dt;
    0,  0, 1,  0;
    0,  0, 0,  1];

B = [0; 0; 0; -g*dt];

mu = zeros(4, 1);
Q = [20, 0, 5, 0;
    0, 20, 0, 5;
    5, 0, 10, 0;
    0, 5, 0, 10];

% initial condition means
xhat = [0; 0; 100; 100];


% initial conditions covariance
P = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 25, 5;
    0, 0, 5, 25];


%% PROBLEM 1
% mean vehicle state and covariance at time t=5

% PART C
% recursively udpates the vehicle state + covariance
% not sure if w (uncertainty) should factor in here.

P_5 = P;
xhat_5 = xhat;
for i=1:5
    xhat_new = A*xhat_5 + B;
    xhat_5 = xhat_new;
    P_new = A*P_5*A.' + Q;
    P_5 = P_new;
end

% PART D
figure(1)
hold on
scatter(xhat_5(1), xhat_5(2), 'b', 'filled')
confidence_ellipse(xhat_5, P_5, 0.1, 'r');
confidence_ellipse(xhat_5, P_5, 0.05, 'r');
ylabel('$x_{2}$', 'interpreter', 'latex');
xlabel('$x_{1}$', 'interpreter', 'latex');
title('90% and 95% Confidence Intervals for Projectile State')
axis equal;

% PART E
xhat_10 = xhat;
P_10 = P;

for i=1:10
    xhat_new = A*xhat_10 + B;
    xhat_10 = xhat_new;
    P_new = A*P_10*A.' + Q;
    P_10 = P_new;
end

xhat_15 = xhat;
P_15 = P;
for i=1:15
    xhat_new = A*xhat_15 + B;
    xhat_15 = xhat_new;
    P_new = A*P_15*A.' + Q;
    P_15 = P_new;
end

figure(2)
hold on
axis equal

scatter(xhat_10(1), xhat_10(2), 'k', 'filled')
confidence_ellipse(xhat_10, P_10, 0.1, 'k');
ylabel('$x_{2}$', 'interpreter', 'latex');
xlabel('$x_{1}$', 'interpreter', 'latex');
title('90% Confidence Interval for Projectile State')

scatter(xhat_15(1), xhat_15(2), 'r', 'filled')
confidence_ellipse(xhat_15, P_15, 0.1, 'r');
ylabel('$x_{2}$', 'interpreter', 'latex');
xlabel('$x_{1}$', 'interpreter', 'latex');
legend('$\hat{x}_{10|0}$', '$P_{10|0}$', '$\hat{x}_{15|0}$', '$P_{15|0}$', 'interpreter', 'latex')
title('90% Confidence Interval for Projectile State')


