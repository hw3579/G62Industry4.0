%% Clear all irrelevant variables, charts, and command-line outputs before starting the program.
close all;
clear all;
clc;

%% Construct The Robot
% Build a robot by creating rigid body objects and associated joint objects.
% Determine their geometric parameters
% Add them to the robot.

%%
% Creates a rigid body tree object named "robot".
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',5);
% Determine the lengths of the robot arm.
L1 = [0.15,-0.1,0.4];
L2 = [0.15,-0.1,0.5];
L3 = [0.2,-0.3,-0.2];
L4 =  [0, 0, -0.6];
%%
% This creates a new rigid body object "R1"
body = rigidBody('R1');
%%% Body
body.Mass=10; % This sets the mass of the body to 10 kg.
body.CenterOfMass=[0 0 0]; % This sets the center of mass of the body to [0 0 0], which assumes that the mass is equally distributed throughout the body.
body.Inertia=[0.27 0.27 0.8 0 0 0]; 
% This sets the inertia tensor of the body. 
% In this case, it assumes that the body is shaped like a cylinder and uses physical formulas to compute the inertia values. 
% The values in the vector correspond to [Ixx Iyy Izz Iyz Ixz Ixy].
%%% Joint
joint = rigidBodyJoint('joint1', 'revolute');%This creates a new revolute joint object "joint1".
joint.PositionLimits=[-5*(pi/180),5*(pi/180)]; % This sets the position limits for the joint to be between -5 and 5 degrees (converted to radians) around the z-axis.
setFixedTransform(joint,trvec2tform([0 0 0]));% Coordinate transformation
joint.JointAxis = [0 0 1];% Sets the axis of rotation for the joint to be the z-axis.
body.Joint = joint; % Connect joint to the body.
addBody(robot, body, 'base');% Adds the body to the robot rigid body tree

%%
%This code creates a rigid body "R2" and adds a revolute joint "joint3" to it. 
%The mass of the body is set to 10 kg, its center of mass is assumed to be distributed equally at [0 0 0], 
% and the inertia tensor is set to [0.27 0.27 0.8 0 0 0] assuming a cylindrical shape for the joint. 
% The joint rotates around the y-axis, with an initial position at L1 and rotation limits of -30 to 30 degrees.
body = rigidBody('R2');

body.Mass=10;
body.CenterOfMass=[0 0 0];
body.Inertia=[0.27 0.27 0.8 0 0 0];

joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform(L1));
joint.JointAxis = [0 1 0];
joint.PositionLimits=[-30*(pi/180),30*(pi/180)];
body.Joint = joint;
addBody(robot, body, 'R1');
%%
%This code creates a rigid body "P1" and adds a prismatic joint "joint2" to it. 
% The mass of the body is set to 1.5 kg, its center of mass is assumed to be distributed equally at [0 0 0], 
% and the inertia tensor is set to [0.07 0.07 0.07 0 0 0] assuming a cubic shape for the joint. 
% The joint moves along the x-axis, with position limits between -0.5 to 0.5 meters. 
% The initial position of the joint is at L2.
body = rigidBody('P1');

body.Mass=1.5; 
body.CenterOfMass=[0 0 0]; 
body.Inertia=[0.07 0.07 0.07 0 0 0]; 

joint = rigidBodyJoint('joint2','prismatic');
joint.PositionLimits=[-0.5,0.5];
setFixedTransform(joint, trvec2tform(L2));
joint.JointAxis = [1,0, 0];
body.Joint = joint;
addBody(robot, body, 'R2');
%%
%This code creates a rigid body "P2" and adds a prismatic joint "joint4" to it. 
% The mass of the body is set to 1.5 kg, its center of mass is assumed to be distributed equally at [0 0 0], 
% and the inertia tensor is set to [0.07 0.07 0.07 0 0 0] assuming a cubic shape for the joint. 
% The joint moves along the y-axis, with position limits between -1 to 1 meters. 
% The initial position of the joint is at L3.
body = rigidBody('P2');

body.Mass=1.5; 
body.CenterOfMass=[0 0 0];  
body.Inertia=[0.07 0.07 0.07 0 0 0]; 

joint = rigidBodyJoint('joint4','prismatic');
setFixedTransform(joint, trvec2tform(L3));
joint.JointAxis = [0 1 0];
joint.PositionLimits=[-1,1]; 
body.Joint = joint;
addBody(robot, body, 'P1');

%%
%This code creates an end effector "tool" and adds a fixed joint "fix1" to it. 
% The mass of the body is set to 1.2 kg, its center of mass is assumed to be distributed equally at [0 0 0], 
% and the inertia tensor is set to [0.002 0.002 0.004 0 0 0] assuming a conical shape for the end effector to solder materials. 
% The joint is fixed and has no degree of freedom, and the initial position of the joint is at L4.
body = rigidBody('tool');

body.Mass=1.2; 
body.CenterOfMass=[0 0 0];
body.Inertia=[0.002 0.002 0.004 0 0 0]; 
 
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform(L4));
body.Joint = joint;
addBody(robot, body, 'P2');

%%
%sets the gravity vector of the robot to [0 0 -9.81]
% which is the acceleration due to gravity in the z-axis direction.
robot.Gravity=[0 0 -9.81]; 
%Display robot and config with properties. 
robot 
showdetails(robot)
figure(1)
show(robot) 


%% open model
sim_model='FourlinkRobot';
open(sim_model);


%% PID control definition
% Defines the filter values
Kfilt1=150;
Kfilt2=150;
Kfilt3=150;
%Joint 3 uses PID control for better steady-state error reduction, 
% while joints 1, 2, and 4 use PD control for faster response times and lower overshooting.


%% Generate traj
% Input the four point data
pointa=[0.5 -0.5 0.1];
pointb=[1 -0.5 0.5];
pointc=[1.0,0.5,0.5];
pointd=[0.5 0.5 0.1];
%The function divides the line segment between two input points into 
% eight equally spaced points (9 total points) and 
% returns a matrix with these points as rows.
traj_ab = divide_points(pointa, pointb);
traj_bc = divide_points(pointb, pointc);
traj_cd = divide_points(pointc, pointd);
traj_da = divide_points(pointd, pointa);
%Combine these four matrices together to generate a 40*3 matrix of traj points
eePos_des_traj=combine_matrices(traj_ab,traj_bc,traj_cd,traj_da);
%Generate time sequences
time_traj=[1:48]';



%% simulate model
tsim=50; %simulation time
t_sample=0.01; %0.01 %simulation time step (sample time)
sim(sim_model); %run the simulation of the model

%% plotting workspace and input/output

workspace;  %display the robot workspace

figure(3) % display the input torques
plot(t_sim, input_sim(:,1), 'b', ...
     t_sim, input_sim(:,2), 'r', ...
     t_sim, input_sim(:,3), 'g', ...
     t_sim, input_sim(:,4), 'm');
grid on; hold on;
title('Input torques vs time');
ylabel('torque (N)');
xlabel('time (sec)');
legend('torque 1', 'torque 2', 'torque 3', 'torque 4');

%%
figure(4) % display the simulate angles vs desired angles
subplot(4,1,1);
plot(t_sim,qs(:,1)*180/pi,'r', t_sim,qs_des(:,1)*180/pi,'--r'); 
grid on; hold on;
legend('angle 1 (deg)',' angle 1 desired (deg)');
ylabel('Angles (deg)'); xlabel('time (sec)');
title('Joint angles vs time');

subplot(4,1,2);
plot(t_sim,qs(:,2)*180/pi,'b', t_sim,qs_des(:,2)*180/pi,'--r'); 
grid on; hold on;
legend('angle 2 (deg)',' angle 2 desired(deg)')
ylabel('Angles (deg)'); xlabel('time (sec)')

subplot(4,1,3);
plot(t_sim,qs(:,3)*180/pi,'g', t_sim,qs_des(:,3)*180/pi,'--r'); 
grid on; hold on;
legend('angle 3 (deg)',' angle 3 desired(deg)')
ylabel('Angles (deg)'); xlabel('time (sec)')

subplot(4,1,4);
plot(t_sim,qs(:,4)*180/pi,'m', t_sim,qs_des(:,4)*180/pi,'--r'); 
grid on; hold on;
legend('angle 4 (deg)',' angle 4 desired(deg)')
ylabel('Angles (deg)'); xlabel('time (sec)')

%%

figure(5) % display the simulate position vs desired position
subplot(3,1,1);
plot(t_sim,eePos_sim(:,1),'b', t_sim,eePos_des(:,1),'--r'); 
grid on; hold on;
title('End effector x position vs time'); 
legend('x','x desired')
ylabel('x position (m)'); xlabel('time (sec)')

subplot(3,1,2);
plot(t_sim,eePos_sim(:,2),'b', t_sim,eePos_des(:,2),'--r'); 
grid on; hold on;
title('End effector y position vs time');
legend('y','y desired')
ylabel('y position (m)'); xlabel('time (sec)')

subplot(3,1,3);
plot(t_sim,eePos_sim(:,3),'b', t_sim,eePos_des(:,3),'--r'); 
grid on; hold on;
title('End effector z position vs time'); 
legend('z','z desired')
ylabel('z position (m)'); xlabel('time (sec)')

%%
figure(6) % display the end effector error
error=abs(eePos_des-eePos_sim);

subplot(3,1,1);
plot(t_sim,error(:,1),'b'); 
grid on; hold on;
title('End effector x position error vs time'); 
ylabel('x position error'); xlabel('time (sec)')

subplot(3,1,2); 
plot(t_sim,error(:,2),'b'); 
grid on; hold on;
title('End effector y position error vs time'); 
ylabel('y position error'); xlabel('time (sec)')

subplot(3,1,3); 
plot(t_sim,error(:,3)); 
grid on; hold on;
title('End effector z position error vs time'); 
ylabel('z position error'); xlabel('time (sec)')

%%
figure(7) %display the joint angle control error

error_jointangle=abs(qs-qs_des);
error_jointangle_percentage=(error_jointangle./qs_des);

subplot(4,1,1); 
plot(t_sim,error_jointangle(:,1),'b'); 
grid on; hold on;
title('joint1 angle error vs time'); 
ylabel('joint1 angle error'); xlabel('time (sec)')

subplot(4,1,2); 
plot(t_sim,error_jointangle(:,2),'b'); 
grid on; hold on;
title('joint2 angle error vs time'); 
ylabel('joint2 angle error'); xlabel('time (sec)')

subplot(4,1,3); 
plot(t_sim,error_jointangle(:,3),'b'); 
grid on; hold on;
title('joint3 angle error vs time'); 
ylabel('joint3 angle error'); xlabel('time (sec)')

subplot(4,1,4); 
plot(t_sim,error_jointangle(:,4),'b'); 
grid on; hold on;
title('joint4 angle error vs time'); 
ylabel('joint4 angle error'); xlabel('time (sec)')



%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot configuration.
Robot_animate_3d
