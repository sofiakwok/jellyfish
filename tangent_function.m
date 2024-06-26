% Figuring out when the function to find beta angle becomes undefined

x = linspace(0, 2, 100);
y = linspace(0, 2, 100);
[x_2, y_2] = meshgrid(x, y);
m_1 = -0.153543;
m_2 = 0.405512;
d = 3.052717;
l = 0.568898;

a = (4*l*m_1 - 4*l*x_2).^2;
b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
top = sqrt(a - 4.*b.*c) - 2*l*m_1 + 2*l.*x_2; 
bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
beta = 2*(atan(0.5*top./bottom));

top = sqrt(abs(a - 4.*b.*c)) - 2*l*m_1 + 2*l.*x_2; 
bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
abs_beta = 2*(atan(0.5*top./bottom));

real_beta = real(beta);
pos_beta = abs(real_beta);
deg_beta = real_beta * 180/3.1415;

figure;
hold on
surf(x_2, y_2, real_beta)
xlabel('X')
ylabel('Y')
zlabel('Z')

figure(2);
hold on
surf(x_2, y_2, abs_beta)
xlabel('X')
ylabel('Y')
zlabel('Z')

%% fin1 beta
%graphing beta over the feasible range that x1, y1 can be at

f = 2.15178; % fin length
d = 3.052717; % steering length
l = 0.56; %servo arm length
r = 0.2519; %0.483608; %rudder length

m_1 = -0.704724;
m_2 = 0.405512;

theta = linspace(0, 3.1415/2, 100); %all trig in radians

x_1 = f*sin(theta);
y_1 = -f*cos(theta);

% calculate beta for alpha to stay at 0 throughout cycle
alpha = 0/180*3.1415;
x_2 = r*sin(theta + alpha) + x_1;
y_2 = -r*cos(theta + alpha) + y_1;
% x_2 = (l + f) * sin(theta);
% y_2 = -(l + f) * cos(theta);

a = (4*l*x_2 - 4*l*m_1).^2;
b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
top = 0.5*sqrt(a - 4.*b.*c) + 2*l*m_1 - 2*l.*x_2; 
bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
beta = 2*(atan(top./bottom));
top_2 = -0.5*sqrt(a - 4.*b.*c) + 2*l*m_1 - 2*l.*x_2;
beta_2 = 2*(atan(top_2./bottom));

deg_beta = beta * 180 / 3.1415;
deg_beta_2 = beta_2 * 180 / 3.1415;

figure(2);
hold on
plot(theta*180/3.1415, deg_beta)
plot(theta*180/3.1415, deg_beta_2)
xlabel('\theta')
ylabel('\beta')

%% fin2 beta
%graphing beta over the feasible range that x1, y1 can be at

f = 2.15178; % fin length
d = 3.052717; % steering length
l = 0.56; %servo arm length
r = 0.2519; %0.483608; %rudder length

m_1 = 0.704724;
m_2 = 0.405512; %2

theta = linspace(0, 3.1415/2, 100); %all trig in radians

x_1 = -f*sin(theta);
y_1 = -f*cos(theta);

% calculate beta for alpha to stay at 0 throughout cycle
alpha = 0/180*3.1415;
x_2 = -r* sin(theta + alpha) + x_1;
y_2 = -r * cos(theta + alpha) + y_1; 
% x_2 = (l + f) * sin(theta);
% y_2 = -(l + f) * cos(theta);

a = (4*l*m_1 - 4*l*x_2).^2;
b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
top = real(0.5*sqrt(a - 4.*b.*c)) - 2*l*m_1 + 2*l.*x_2; 
bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
beta = 2*(atan(top./bottom));
top_2 = -0.5*sqrt(a - 4.*b.*c) - 2*l*m_1 + 2*l.*x_2;
beta_2 = 2*(atan(top_2./bottom));

deg_beta = beta * 180 / 3.1415;
deg_beta_2 = beta_2 * 180 / 3.1415;

figure(3);
hold on
plot(theta*180/3.1415, deg_beta)
plot(theta*180/3.1415, deg_beta_2)
xlabel('\theta')
ylabel('\beta')

%% fin1_2 beta
%graphing beta over the feasible range that x1, y1 can be at

f = 2.15178; % fin length
d = 2.9; % steering length
l = 0.56; %servo arm length
r = 0.52; %rudder length

m_1 = -0.185039;
m_2 = 0.405512;

theta = linspace(0, 3.1415/2, 100); %all trig in radians

x_1 = f*sin(theta);
y_1 = -f*cos(theta);

% calculate beta for alpha to stay at 0 throughout cycle
alpha = 0/180*3.1415;
x_2 = r*sin(theta + alpha) + x_1;
y_2 = -r*cos(theta + alpha) + y_1;
% x_2 = (l + f) * sin(theta);
% y_2 = -(l + f) * cos(theta);

a = (4*l*x_2 - 4*l*m_1).^2;
b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
top = 0.5*sqrt(a - 4.*b.*c) + 2*l*m_1 - 2*l.*x_2; 
bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
beta = 2*(atan(top./bottom));
top_2 = -0.5*sqrt(a - 4.*b.*c) + 2*l*m_1 - 2*l.*x_2;
beta_2 = 2*(atan(top_2./bottom));

deg_beta = beta * 180 / 3.1415;
deg_beta_2 = beta_2 * 180 / 3.1415;

figure(2);
hold on
plot(theta*180/3.1415, deg_beta)
plot(theta*180/3.1415, deg_beta_2)
xlabel('\theta')
ylabel('\beta')

%% fin2_2 beta
%graphing beta over the feasible range that x1, y1 can be at

f = 2.15178; % fin length
d = 2.85; % steering length
l = 0.56; %servo arm length
r = 0.52; %rudder length

m_1 = 0.185039;
m_2 = 0.405512; %2

theta = linspace(0, 3.1415/2, 100); %all trig in radians

x_1 = -f*sin(theta);
y_1 = -f*cos(theta);

% calculate beta for alpha to stay at 0 throughout cycle
alpha = 0/180*3.1415;
x_2 = -r* sin(theta + alpha) + x_1;
y_2 = -r * cos(theta + alpha) + y_1; 
% x_2 = (l + f) * sin(theta);
% y_2 = -(l + f) * cos(theta);

a = (4*l*m_1 - 4*l*x_2).^2;
b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
top = real(0.5*sqrt(a - 4.*b.*c)) - 2*l*m_1 + 2*l.*x_2; 
bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
beta = 2*(atan(top./bottom));
top_2 = -0.5*sqrt(a - 4.*b.*c) - 2*l*m_1 + 2*l.*x_2;
beta_2 = 2*(atan(top_2./bottom));

deg_beta = beta * 180 / 3.1415;
deg_beta_2 = beta_2 * 180 / 3.1415;

figure(3);
hold on
plot(theta*180/3.1415, deg_beta)
plot(theta*180/3.1415, deg_beta_2)
xlabel('\theta')
ylabel('\beta')

%% fin1 beta right angle
%graphing beta over the feasible range that x1, y1 can be at

f = 2.15178; % fin length
d = 2.45; % steering length for servo attachment
l = 0.568898; %servo arm length
r = 0.2481; %rudder length

m_1 = -0.311024;
m_2 = 1.165354;

theta = linspace(0, 3.1415/2, 100); %all trig in radians

x_1 = f*sin(theta);
y_1 = -f*cos(theta);

figure(2);
clf;
hold on
% calculate beta for alpha to stay at 0 throughout cycle
for i = 1:1
    alpha = (i-1) * 10 / 180 * 3.1415;
    x_2 = r*sin(theta + alpha + 3.1415/2) + x_1;
    y_2 = -r*cos(theta + alpha + 3.1415/2) + y_1;
    % x_2 = (l + f) * sin(theta);
    % y_2 = -(l + f) * cos(theta);
    
    a = (4*l*x_2 - 4*l*m_1).^2;
    b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
    c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
    top = 0.5*sqrt(a - 4.*b.*c) + 2*l*m_1 - 2*l.*x_2; 
    bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
    top_2 = -0.5*sqrt(a - 4.*b.*c) + 2*l*m_1 - 2*l.*x_2;
    beta_2 = 2*(atan(top_2./bottom));
    beta = 2*(atan(top./bottom));
    deg_beta = real(beta) * 180 / 3.1415;
    deg_beta_2 = beta_2 * 180 / 3.1415;
    plot(theta*180/3.1415, deg_beta)
    plot(theta*180/3.1415, deg_beta_2, "--")
end
plot(theta*180/3.1415, theta*0)

xlabel('\theta')
ylabel('\beta')

%% fin2 beta right angle
%graphing beta over the feasible range that x1, y1 can be at

f = 2.153604; % fin length
d = 2.45; % steering length for servo attachment
l = 0.568898; %servo arm length
r = 0.2481; %rudder length

m_1 = 0.311024;
m_2 = 1.165354; %2

theta = linspace(0, 3.1415/2, 100); %all trig in radians

x_1 = -f*sin(theta);
y_1 = -f*cos(theta);

figure(3);
clf;
hold on
% calculate beta for alpha to stay at 0 throughout cycle
for i = 1:2
    alpha = (i-1) * 10 / 180 * 3.1415;
    x_2 = -r* sin(theta + alpha + 3.1415/2) + x_1;
    y_2 = -r * cos(theta + alpha + 3.1415/2) + y_1; 
    % x_2 = (l + f) * sin(theta);
    % y_2 = -(l + f) * cos(theta);
    
    a = (4*l*m_1 - 4*l*x_2).^2;
    b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
    c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
    top = 0.5*sqrt(a - 4.*b.*c) - 2*l*m_1 + 2*l.*x_2; 
    bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
    top_2 = real(-0.5*sqrt(a - 4.*b.*c)) + 2*l*m_1 - 2*l.*x_2;
    beta_2 = 2*(atan(top_2./bottom));
    beta = 2*(atan(top./bottom));
    deg_beta = real(beta) * 180 / 3.1415;
    deg_beta_2 = beta_2 * 180 / 3.1415;
    plot(theta*180/3.1415, deg_beta)
    plot(theta*180/3.1415, deg_beta_2, "--")
end
plot(theta*180/3.1415, theta*0)
xlabel('\theta')
ylabel('\beta')
