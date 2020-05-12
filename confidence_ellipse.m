function [] = confidence_ellipse(mu,sigma, alpha, color)
%CONFIDENCE_ELLIPSE Plot confidence ellipses for bi-variate gaussian
%   Input argument: Mean, covariance, alpha, color

%% Sanity checks

% Sigma must be symmetric and PSD
% if  ~ isequal(sigma,sigma')
%    error('Covariance matrix is not symmetric') 
% end

if any(eig(sigma)<=0) || ~isreal(eig(sigma))
    error('Covariance matrix is not positive definite')
end

% alpha must be between 0 and 1
if alpha<=0 || alpha >=1
    error('Alpha should be strictly between 0 and 1')
end

%% Set up 

% Size of ellipse based on alpha (refer to Recitation 10)
z = chi2inv(alpha,2);

[v,d] = eigs(sigma);

% eigenvalues
lambda_1 = d(1,1);
lambda_2 = d(2,2);

% eigenvectors
e1 = v(:,1);
e2= v(:,2);

% orientation of the ellipse's major axis 
theta_0 = atan2(e1(2),e1(1));

%% Plot
N = 50; % resolution of the ellipse
theta = linspace(0, 2*pi, N)';

% use polar coordinates to generate points on the ellipse
x = mu(1) + sqrt(lambda_1 * z)* cos(theta) * cos(theta_0) - sqrt(lambda_2 * z)* sin(theta)*sin(theta_0);
y = mu(2) + sqrt(lambda_1 * z)* cos(theta) * sin(theta_0) + sqrt(lambda_2 * z)* sin(theta)*cos(theta_0);

hold on;
plot(x,y, color);

end

