% Note: project references robot.m

%% ENVIRONMENT SETUP
clear all;
close all;
clc;

%% Question 1

% QUESTION BODY
load('S_letter_path.mat')
% plot data
plot(Sls(1,:), Sls(2,:));
hold on
xlabel('x');
ylabel('y');

% plot outward normal
 [rows, cols] = size(Sls);
 normal = zeros(2, cols);
 for i = 1:cols - 1
     deltax = Sls(1, i + 1) - Sls(1, i);
     deltay = Sls(2, i + 1) - Sls(2, i);
     quiver(Sls(1, i), Sls(2, i), -deltay, deltax)
     normal(1, i) = Sls(1, i) - deltay;
     normal(2, i) = Sls(2, i) + deltax;
 end
normal(1, end) = 2.5649;
normal(2, end) = 1.1232;
hold off

% calculate path length array
pathLength = zeros(2, cols);
cumulativex = 0;
cumulativey = 0;
for i = 2:cols
    xDistance = Sls(1, i) - Sls(1, i - 1);
    yDistance = Sls(2, i) - Sls(2, i - 1);
    cumulativex = cumulativex + xDistance;
    cumulativey = cumulativey + yDistance;
    pathLength(1, i) = cumulativex;
    pathLength(2, i) = cumulativey;
end

%% Question 2

q = -pi + (2*pi)*rand(3, 1);
bot = robot();
bot.q = q;
bot.forwards();

%% Question 3
% ENVIRONMENT SETUP
clc;
clear all;
close all;

% QUESTION BODY
q = -pi + (2*pi)*rand(3, 1);
bot = robot();
bot.q = q;
bot.forwards();
bot.reverse();
bot.compare();

%% Question 4
% Note: must run Q1 to obtain necessary variables in workspace for this to
% work

% QUESTION BODY
load('S_letter_path.mat')
hold on
for i = 1:cols
    % Reverse kinematics
    % define T
    T = eye(4);
    % Set P values
    T(1, 4) = Sls(1, i);
    T(2, 4) = Sls(2, i);
    % set R values
    xT = Sls(1, i) - normal(1, i);
    yT = Sls(2, i) - normal(2, i);
    qT = atan(xT / yT);
    T(1, 1) = cos(qT);
    T(1, 2) = -sin(qT);
    T(2, 1) = sin(qT);
    T(2, 2) = cos(qT);

    q4bot = robot();
    q4bot.T = T;
    q4bot.reverse();


    % Plot arm
    % ARM CONFIGURATION 1
    q = q4bot.qsol(1,:);
    coords = zeros(2, 4);
    % arm start point
    coords(1, 1) = 0;
    coords(1, 2) = 0;
    % arm joint 1
    coords(1, 2) = 1.5*cos(q(1));
    coords(2, 2) = 1.5*sin(q(1));
    % arm joint 2
    coords(1, 3) = coords(1, 2) + 1.5*cos(q(1) + q(2));
    coords(2, 3) = coords(2, 2) + 1.5*sin(q(1) + q(2));
    % arm joint 3 (end)
    coords(1, 4) = coords(1, 3) + 0.5*cos(sum(q));
    coords(2, 4) = coords(2, 3) + 0.5*sin(sum(q));

    % plot results
    plot(coords(1,:), coords(2,:))
    
    % ARM CONFIGURATION 2
    q = q4bot.qsol(2,:);
    % arm start point
    coords(1, 1) = 0;
    coords(1, 2) = 0;
    % arm joint 1
    coords(1, 2) = 1.5*cos(q(1));
    coords(2, 2) = 1.5*sin(q(1));
    % arm joint 2
    coords(1, 3) = coords(1, 2) + 1.5*cos(q(1) + q(2));
    coords(2, 3) = coords(2, 2) + 1.5*sin(q(1) + q(2));
    % arm joint 3 (end)
    coords(1, 4) = coords(1, 3) + 0.5*cos(sum(q));
    coords(2, 4) = coords(2, 3) + 0.5*sin(sum(q));

    % plot results
    plot(coords(1,:), coords(2,:))
end
hold off
axis equal

%% Question 5
% Note: must run Q1 to obtain necessary variables in workspace for this to
% work

% QUESTION BODY
load('S_letter_path.mat')
hold on
oldq1 = [0 0 0];
qmoved1 = [0 0 0];
oldq2 = [0 0 0];
qmoved2 = [0 0 0];
timepassed1 = 0;
timepassed2 = 0;
for i = 1:cols
    % Reverse kinematics
    % define T
    T = eye(4);
    % Set P values
    T(1, 4) = Sls(1, i);
    T(2, 4) = Sls(2, i);
    % set R values
    xT = Sls(1, i) - normal(1, i);
    yT = Sls(2, i) - normal(2, i);
    qT = atan(xT / yT);
    T(1, 1) = cos(qT);
    T(1, 2) = -sin(qT);
    T(2, 1) = sin(qT);
    T(2, 2) = cos(qT);

    q4bot = robot();
    q4bot.T = T;
    q4bot.reverse();
    
    % Plot arm
    % ARM CONFIGURATION 1
    q = q4bot.qsol(1,:);
    
    % enforce speed constraint
    qmoved1 = qmoved1 + q - oldq1;
    if max(qmoved1) > 1
        timepassed1 = max(qmoved1);
        qmoved1 = [0 0 0];
    end
    oldq1 = q;
    
    coords = zeros(2, 4);
    % arm start point
    coords(1, 1) = 0;
    coords(1, 2) = 0;
    % arm joint 1
    coords(1, 2) = 1.5*cos(q(1));
    coords(2, 2) = 1.5*sin(q(1));
    % arm joint 2
    coords(1, 3) = coords(1, 2) + 1.5*cos(q(1) + q(2));
    coords(2, 3) = coords(2, 2) + 1.5*sin(q(1) + q(2));
    % arm joint 3 (end)
    coords(1, 4) = coords(1, 3) + 0.5*cos(sum(q));
    coords(2, 4) = coords(2, 3) + 0.5*sin(sum(q));

    % plot results
    plot(coords(1,:), coords(2,:))
    
    % ARM CONFIGURATION 2
    q = q4bot.qsol(2,:);
    qmoved2 = qmoved2 + q - oldq2;
    
    % enforce speed constraint
    if max(qmoved2) > 1
        timepassed2 = max(qmoved2);
        qmoved2 = [0 0 0];
    end
    oldq2 = q;
    
    % arm start point
    coords(1, 1) = 0;
    coords(1, 2) = 0;
    % arm joint 1
    coords(1, 2) = 1.5*cos(q(1));
    coords(2, 2) = 1.5*sin(q(1));
    % arm joint 2
    coords(1, 3) = coords(1, 2) + 1.5*cos(q(1) + q(2));
    coords(2, 3) = coords(2, 2) + 1.5*sin(q(1) + q(2));
    % arm joint 3 (end)
    coords(1, 4) = coords(1, 3) + 0.5*cos(sum(q));
    coords(2, 4) = coords(2, 3) + 0.5*sin(sum(q));

    % plot results
    plot(coords(1,:), coords(2,:))
end
hold off
axis equal

%% Question 6

% setup step, only done to have a jacobian and two sets of q values
q = -pi + (2*pi)*rand(3, 1);
bot = robot();
bot.q = q;
bot.forwards();
bot.reverse();

bot1 = robot();
bot1.q = bot.qsol(1,:);
bot1.forwards()
J1 = bot1.J;

bot2 = robot();
bot2.q = bot.qsol(2,:);
bot2.forwards()
J2 = bot2.J;

% QUESTION BODY
qdot = [1; 1; 1];
% configuration 1 speed
syms x
eqn1 = qdot == inv(J1)*x;
[x] = solve([eqn1], [x]);
x = eval(x);

syms y
eqn2 = qdot == inv(J2)*y;
[y] = solve([eqn2], [y]);
y = eval(y);


max([x, y])