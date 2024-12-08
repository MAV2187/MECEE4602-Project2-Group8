robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);

% Link lengths
L1 = 0; % Base link 
L2 = 15.375; % Offset between prismatic joint and rotational joint
L3 = 9.5; % Rotational link length
L4 = 7.325; % End effector offset

% Base to Link 1
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint, trvec2tform([0 0 L1])); 
joint.JointAxis = [0 0 1]; 
body.Joint = joint;
addBody(robot, body, 'base');

% Link 1 to Link 2 
body = rigidBody('link2');
joint = rigidBodyJoint('joint2', 'prismatic');
setFixedTransform(joint, trvec2tform([0 0 0]));  
joint.JointAxis = [0 0 1]; 
joint.PositionLimits = [0 20]; 
body.Joint = joint;
addBody(robot, body, 'link1');

% Link 2 to Link 3 
body = rigidBody('link3');
joint = rigidBodyJoint('joint3', 'revolute');
setFixedTransform(joint, trvec2tform([L2 0 0])); 
joint.JointAxis = [0 0 1]; 
body.Joint = joint;
addBody(robot, body, 'link2');

% Link 3 to Link 4 
body = rigidBody('link4');
joint = rigidBodyJoint('joint4', 'revolute');
setFixedTransform(joint, trvec2tform([L3 0 0])); 
joint.JointAxis = [0 1 0]; 
body.Joint = joint;
addBody(robot, body, 'link3');

% Link 4 to End Effector 
body = rigidBody('tool');
joint = rigidBodyJoint('fix1', 'fixed');
setFixedTransform(joint, trvec2tform([L4 0 0])); 
body.Joint = joint;
addBody(robot, body, 'link4');

showdetails(robot);

%%
% Motion profiles for each joint
t = (0:0.05:10)'; % Time
count = length(t);

% Rotational motion for theta1
theta1_trajectory = cubictrajectory(0, 1.53, 0, 0, 20, count); 

% Prismatic motion for d2
d2_trajectory = cubictrajectory(2, 17, 0, 0, 20, count);

% Rotational motion for theta3
theta3_trajectory = cubictrajectory(0, -2.05, 0, 0, 20, count); 

% Rotational motion for theta4
theta4_trajectory = cubictrajectory(0, 1.57, 0, 0, 20, count); 

qs = zeros(count, 4); 

% Populate joint configurations (combine motions for d2, theta3, and theta4)
for i = 1:count
    qs(i, :) = [theta1_trajectory(i), d2_trajectory(i), theta3_trajectory(i), theta4_trajectory(i)];
end
%%
% Visualization
figure;
show(robot, qs(1, :)');
view(3); % 3D view
ax = gca;
ax.Projection = 'orthographic';
hold on;
grid on;

% Adjust animation boundaries
axis([-20 40 -20 40 0 20]); % x-y-z boundaries

% Initialize path line
endEffectorPos = zeros(count, 3); % To store end-effector positions
pathLine = plot3(0, 0, 0, 'r', 'LineWidth', 3); % Path line plot

% Animate the robot
framesPerSecond = 30;
r = rateControl(framesPerSecond);
for i = 1:count
    % Show robot
    show(robot, qs(i, :)', 'PreservePlot', false);
    
    % Compute and store end-effector position
    tform = getTransform(robot, qs(i, :)', 'tool', 'base');
    endEffectorPos(i, :) = tform2trvec(tform); % Extract translation (x, y, z)
    
    % Update path line
    set(pathLine, 'XData', endEffectorPos(1:i, 1), ...
                  'YData', endEffectorPos(1:i, 2), ...
                  'ZData', endEffectorPos(1:i, 3));
    
    drawnow;
    waitfor(r);
end
%%
figure;
plot(theta1_trajectory,t);
hold on;
plot(d2_trajectory,t);
plot(theta3_trajectory,t);
plot(theta4_trajectory,t);
legend('theta1','d2','theta3','theta4');
hold off;
% Function for cubic trajectory
function q = cubictrajectory(q_initial, q_final, q_velocity_initial, q_velocity_final, t, n)
    % Time vector
    T = linspace(0, t, n);
    
    % Coefficients for cubic trajectory
    q_conditions = [q_initial; q_final; q_velocity_initial; q_velocity_final];
    time_matrix = [0, 0, 0, 1; ...
                   t^3, t^2, t, 1; ...
                   0, 0, 1, 0; ...
                   3*t^2, 2*t, 1, 0];
    coefficients = time_matrix \ q_conditions;
    
    % Generate trajectory
    q = coefficients(1)*T.^3 + coefficients(2)*T.^2 + coefficients(3)*T + coefficients(4);
end

