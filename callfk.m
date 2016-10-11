%EE 209AS Lab 1 Forward Kinematics 
% Tzu-Wei Chuang, Xuerui Yan

%Transformation_01: d_1 = d_2 = d_3 = 5 cm, theta = 45 degree. 
w = fk(0, 0, 5, 0);
x = fk(0, 0, 5, 0);
y = fk(0, 0, 5, 0);
z = fk(0, 90, 0, 45);

fkresult = w*x*y*z

