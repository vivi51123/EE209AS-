%EE 209AS Lab 1 FK Function
% Tzu-Wei Chuang, Xuerui Yan 

function y = fk(a, alpha, d, theta) 
y = [cos(theta*pi/180), -sin(theta*pi/180), 0, a; 
    sin(theta*pi/180) * cos(alpha*pi/180), cos(theta*pi/180) * cos(alpha*pi/180), -sin(alpha*pi/180), -d * sin(alpha*pi/180); 
    sin(theta*pi/180) * sin(alpha*pi/180), cos(theta*pi/180) * sin(alpha*pi/180), cos(alpha*pi/180) d * cos(alpha*pi/180);
    0, 0, 0, 1];
end 
