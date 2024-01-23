% Figuring out when the function to find beta angle becomes undefined

x = linspace(-2, 2, 100);
y = linspace(-2, 2, 100);
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

figure;
hold on
surf(x_2, y_2, real(beta))
