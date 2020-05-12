dt = 1; % timestep (1s)
g = 9.81; % gravity (m/s^2)

A = [1, 0, dt, 0;
    0,  1, 0,  dt;
    0,  0, 1,  0;
    0,  0, 0,  1];

B = [0; 0; 0; -g*dt];

C = [1, 0, 0, 0;
    0, 1, 0, 0];

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

%% PROBLEM 3

%%A: Generate "ground truth" of projectile
%sample initial condition
x = zeros(100, 4);

%x(3) and x(4) are sampled from Gaussians with mean 100, variance 25, and
%correlation coefficient 0.2
x(1, 3) = normrnd(100, 5);
%derive x(4) using marginal distribution conditioned on x(3)
x(1, 4) = normrnd(100 + 0.2*(x(1,3)-100), sqrt((1-0.2^2)*25));

%propagate state over time until projectile hits ground
ind=1;
while x(ind, 2) >= 0
    ind = ind+1;
    %propagate state without noise
    x(ind,1) = x(ind-1, 1) + dt*x(ind-1,3);
    x(ind,2) = x(ind-1, 2) + dt*x(ind-1,4);
    x(ind,3) = x(ind-1, 3);
    x(ind,4) = x(ind-1, 4) - dt*g;
    %sample noise
    w = zeros(1, 4);
    w(1) = normrnd(0, sqrt(Q(1,1)));
    w(2) = normrnd(0, sqrt(Q(2,2)));
    %derive w(3) using marginal conditioned on w(1), and w(4) conditioned
    %on w(2)
    rho_w31 = Q(3,1)/(sqrt(Q(3,3))*sqrt(Q(1,1)));
    rho_w42 = Q(4,2)/(sqrt(Q(4,4))*sqrt(Q(2,2)));
    w(3) = normrnd(0 + rho_w31*(w(1)), sqrt((1-rho_w31^2)*Q(1,1)));
    w(4) = normrnd(0 + rho_w42*(w(2)), sqrt((1-rho_w42^2)*Q(2,2))); 
    %add noise
    x(ind, 1) = x(ind,1) + w(1);
    x(ind, 2) = x(ind,2) + w(2);
    x(ind, 3) = x(ind,3) + w(3);
    x(ind, 4) = x(ind,4) + w(4);
end
%concatenate x to appropriate length
x = x(1:ind, :);
%save x -- DO NOT UNCOMMENT OR YOU WILL OVERRIDE GROUNDTRUTH.MAT
%save('groundtruth.mat', 'x');
%plot x-position vs y-position
load('groundtruth.mat');
close all
figure()
plot(x(:,1), x(:,2));
xlabel("X-position (m)");
ylabel("Y-position (m)");
ylim([0 max(x(:,2))+20]);
title("Projectile Trajectory: Ground Truth");

%%B: Generate radar observations
%load ground truth
load('groundtruth.mat');
R = [9 3; 3 9];
size_x = size(x);
y = zeros(size_x(1), 2);
%Loop over ground truth and create radar observations
for ind=1:size_x
    y(ind, 1) = x(ind,1);
    y(ind, 2) = x(ind,2);
    %generate radar noise
    r = zeros(2,1);
    r(1) = normrnd(0, sqrt(R(1,1)));
    %r(2) is found from marginal of r(1)
    rho_r12 = R(1, 2)/(sqrt(R(1,1))*sqrt(R(2,2)));
    r(2) = normrnd(0 + rho_r12*r(1), sqrt((1-rho_r12^2)*R(2,2)));
    y(ind,1) = y(ind,1) + r(1);
    y(ind,2) = y(ind,2) + r(2);
end
%save y -- DO NOT UNCOMMENT OR YOU WILL OVERRIDE RADAR.MAT
%save('radar.mat', 'y');
load('radar.mat'); 
figure()
plot(y(:,1), y(:,2));
xlabel("X-position (m)");
ylabel("Y-position (m)");
ylim([0 max(x(:,2))+20]);
title("Projectile Trajectory: Radar Observations");


%%C: Kalman filter
%load radar measurements
load('radar.mat'); 

% position after 10 timesteps
xhat_10 = xhat;
P_10 = P;

for i=0:10
    % filtering
    xhat_new = A*xhat_10 + B
    P_new = A*P_10*A.' + Q;
    
    % prediction
    yhat_new = y(i+1,:).' - C * xhat_new;
    S = C*P_new*C.' + R;
    K = P_new*C.'*S^(-1);
    xhat_updated = xhat_new + K * yhat_new;
    P_updated = (eye(4) - K * C)* P_new;
    
    xhat_10 = xhat_updated;
    P_10 = P_updated;
    
end

xhat_10

% figure()
hold on
scatter(xhat_10(1), xhat_10(2), 'k', 'filled')
confidence_ellipse(xhat_10, P_10, 0.9, 'k');
ylabel('$x_{2}$', 'interpreter', 'latex');
xlabel('$x_{1}$', 'interpreter', 'latex');
title('90% Confidence Interval for Projectile State')

% D: Broken Radar Predictor
for i=11:25
    % filtering
    xhat_new = A*xhat_10 + B;
    P_new = A*P_10*A.' + Q;
    
    % no prediction step: radar is broken
    % essentially repeat procedure from step 1
    
    xhat_10 = xhat_new;
    P_10 = P_new;
    
    if xhat_10(2) < 0
        xhat_10(2);
        time_stopped = i;
        break
    end
end

% figure()
scatter(xhat_10(1), xhat_10(2), 'r', 'filled')
confidence_ellipse(xhat_10, P_10, 0.9, 'r');
ylabel('$x_{2}$', 'interpreter', 'latex');
xlabel('$x_{1}$', 'interpreter', 'latex');
title('90% Confidence Interval for Projectile State')
legend('Trajectory', '$t=10$', '$t=10$ CI', '$t=20$', '$t=20$ CI', 'interpreter', 'latex')

