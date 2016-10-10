%EE 209AS Lab 1 Forward Kinematics 
% Tzu-Wei Chuang, Xuerui Yan

%Transformation_01: d_0 = 15 cm, theta = 45 degree. 
w = fk(0, 0, 15, 0);
x = fk(0, 0, 15, 0);
y = fk(0, 0, 15, 0);
z = fk(0, 0, 0, sind(45));

fkresult = w*x*y*z

