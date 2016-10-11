%EE 209AS Lab 1 Inverse Kinematics 
% Tzu-Wei Chuang, Xuerui Yan

%End effector initial position 
e_initial = fk(0, 0, 0, 0)*fk(0, 0, 0, 0)*fk(0, 0, 0, 0)*fk(0, 90, 0, 0);

%Our desired end effector position
e_desired = fk(0, 0, 5, 0)*fk(0, 0, 5, 0)*fk(0, 0, 5, 0)*fk(0, 90, 0, 45)

%The difference between each step of the transformation of the end effector
difference_e = e_desired(1:3, 4) - e_initial(1:3, 4);

%A copy of the intial position to get into the loop
e_previous = e_initial(1:3, 4);

%An zero matrix to start with for solving the inverse kinematics
q_test = zeros(4, 1);
   
while (norm(difference_e) > 1.0e-30)
   
   %The magnitude of the difference between each step of transformation
   error = norm(difference_e);
   
   %End effector frame with variables
   syms d_1 d_2 d_3 theta_4;
   W = fk(0, 0, d_1, 0);
   X = fk(0, 0, d_2, 0);
   Y = fk(0, 0, d_3, 0);
   Z = fk(0, 0, 0, theta_4);
   fk_variable = W*X*Y*Z;
   
   %Calculate Jacobian with the differential equation  
   J = [diff(fk_variable(1,4), d_1), diff(fk_variable(1,4), d_2), diff(fk_variable(1,4), d_3), diff(fk_variable(1,4), theta_4);
        diff(fk_variable(2,4), d_1), diff(fk_variable(2,4), d_2), diff(fk_variable(2,4), d_3), diff(fk_variable(2,4), theta_4);  
        diff(fk_variable(3,4), d_1), diff(fk_variable(3,4), d_2), diff(fk_variable(3,4), d_3), diff(fk_variable(3,4), theta_4)]
   
   %Using pseudoinverse in matlab to calculate the inverse Jacobian matrix
   J_inv = pinv(J);
   
   difference_e = e_desired(1:3,4) - e_previous;
   
   derivative_q = J_inv * difference_e;
   
   %Update the current generalized coordinates
   q_test = q_test + derivative_q;
   
   %Update the new end effector position from last step
   e_previous = fk(0, 0, q_test(1), 0) * fk(0, 0, q_test(2), 0) * fk(0, 0, q_test(3), 0)* fk(0, 0, 0, q_test(4));
   
   e_previous = e_previous(1:3, 4);
   
end    