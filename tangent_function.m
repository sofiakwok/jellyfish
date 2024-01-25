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

%% 
%graphing beta over the feasible range that x1, y1 can be at

f = 2.15; % fin length
d = 3.052717; % steering length
l = 0.568898; %rudder length

m_1 = -0.153543;
m_2 = 0.405512;

theta = linspace(0, 3.1415/2, 100); %all trig in radians

x_1 = -f*sin(theta);
y_1 = f*cos(theta);

% calculate beta for alpha to stay at 0 throughout cycle
% x_2 = l* sin(theta + alpha) + x_1;
% y_2 = -l * cos(theta + alpha) + y_1;
x_2 = (l + f) * sin(theta);
y_2 = -(l + f) * cos(theta);

% a = (4*l*m_1 - 4*l*x_2).^2;
a = (4*l*x_2 - 4*l*m_1).^2;
b = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
c = d^2 - l^2 - 2*l*m_2 + 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
top = sqrt(a - 4.*b.*c) - 2*l*m_1 + 2*l.*x_2; 
bottom = d^2 - l^2 + 2*l*m_2 - 2*l*y_2 - m_1^2 + 2*m_1*x_2 - m_2^2 + 2*m_2*y_2 - x_2.^2 - y_2.^2;
beta = 2*(atan(0.5*top./bottom));

figure(3);
hold on
plot(theta, beta)
xlabel('\theta')
ylabel('\beta')


