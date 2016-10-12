% Choose a desired position of the end-effector, which is reachable
e_d = [1 0 0 0
       0 1 0 45
       0 0 1 0
       0 0 0 1];

% Choose a initial position of the end-effector, which is reachable
e_c = [1 0 0 45
       0 1 0 0
       0 0 1 0
       0 0 0 1];
   
% Define the variables for the joint space
syms theta_1;
syms theta_2;
syms theta_3;
syms theta_4;

% Find the pose of the end-effector as a function of q, which are all theta
% in my case 
end_effector = df_matrix(0,     0,     0,  theta_1)...
             * df_matrix(20,    0,     0,  theta_2)...
             * df_matrix(20,    0,     0,  theta_3)...
             * df_matrix(5,     0,     0,  theta_4);

% Calculate the Jacobian of the system
jk_jacobian = [diff(end_effector(1,4),theta_1),diff(end_effector(1,4),theta_2),diff(end_effector(1,4),theta_3),diff(end_effector(1,4),theta_4);
               diff(end_effector(2,4),theta_1),diff(end_effector(2,4),theta_2),diff(end_effector(2,4),theta_3),diff(end_effector(2,4),theta_4);
               diff(end_effector(3,4),theta_1),diff(end_effector(3,4),theta_2),diff(end_effector(3,4),theta_3),diff(end_effector(3,4),theta_4)];
f_jk_jacobian = symfun(jk_jacobian, [theta_1 theta_2 theta_3 theta_4]);


%Find the initial difference of the desired pose and initial pose of the
%end effector
delta = e_d - e_c;
error = norm(delta(1:3,4));
q = [0;0;0;0];

% Doing the iteration to find all the values of theta
hold on
while (error>0.1)
    
    jk_jacobian_ans = f_jk_jacobian(q(1),q(2),q(3),q(4));
    
    inv_jk_jacobian = pinv(jk_jacobian_ans);
    dq=inv_jk_jacobian*(delta(1:3,4));
 
    q(1) = q(1) + 1*double(dq(1));
    q(2) = q(2) + 1*double(dq(2));
    q(3) = q(3) + 1*double(dq(3));
    q(4) = q(4) + 1*double(dq(4));
    
    % Use Forward Kinematics to find the operation space of the end
    % effector
    e_c = df_matrix(0,    0,   0,  q(1))...
        * df_matrix(20,   0,   0,  q(2))...
        * df_matrix(20,   0,   0,  q(3))...
        * df_matrix(5,    0,   0,  q(3));

    scatter(e_c(1,4),e_c(2,4),'filled','r');

    delta = e_d - e_c;
    error = norm(delta(1:3,4));


% % %     plot([0 20*cos(q(1))],[0 20*sin(q(1))],'LineWidth',2);
% % %     plot([20*cos(q(1)) 20*cos(q(1))+20*cos(q(1)+q(2))],[20*sin(q(1)) 20*sin(q(1))+20*sin(q(1)+q(2))],'LineWidth',2);
% % %     plot([20*cos(q(1))+20*cos(q(1)+q(2)) 20*cos(q(1))+cos(q(1)+q(2))+5*cos(q(1)+q(2)+q(3))],[20*sin(q(1))+20*sin(q(1)+q(2)) 20*sin(q(1))+20*sin(q(1)+q(2))+5*sin(q(1)+q(2)+q(3))],'LineWidth',2); 
end



