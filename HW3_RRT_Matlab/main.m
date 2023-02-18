clear all; clc; close all;

% Robot Info.   (x, y, z) of link


% x, y, z, roll, pitch, yaw
global q_goal;
% Map 0, 1
% q_init = [0, -20, 0, deg2rad(0), deg2rad(0), deg2rad(0)];
% q_goal = [0, 20, 0, deg2rad(90), deg2rad(0), deg2rad(0)];

% Map 2, 3
q_init = [-2, 0, 0, deg2rad(0), deg2rad(0), deg2rad(0)];
q_goal = [3, 12, 10, deg2rad(0), deg2rad(0), deg2rad(0)];

% graph, (1, 2) => (init, goal)
global G;
G.map = [1]; 
G.qlist = [q_init];

% And so on
global gamma;
global epsilon;
gamma = 0.5;
epsilon = 1;
check_start_idx = 1;

% Env
robot = Robot1();
init_robot = Robot1();
obss=Map3();

Kinematics(init_robot, robot, q_init);
Simulate(robot, obss, false)
Draw_Init_Goal(init_robot, robot, q_init, q_goal)
pause()

while(true)
    q_rand = SamplingQ(init_robot, robot, obss);
    
    min_idx = Find_q_near(q_rand);
    q_near = G.qlist(min_idx, :);

    Expansion_from_q_near(init_robot, robot, obss, q_near, q_rand);

    [goalFlag, check_start_idx] = CheckGoal(init_robot, robot, obss, check_start_idx);
   
    if goalFlag == false                    % goal과 연결 시.
        nodeMap = MakeGraph();

        if height(nodeMap.Nodes) == 0
            continue;
        end

        % plot(nodeMap)
        
        [path, d] = shortestpath(nodeMap, 1, height(nodeMap));            % 1: init, 2: goal
        if d ~= Inf
            break
        end
    end
end
disp("END_Planning")

%%
q_goal_idx = Find_q_idx(q_goal);
[path, d] = shortestpath(nodeMap, 1, q_goal_idx);
Draw_Init_Goal(init_robot, robot, q_init, q_goal)
for i=1:length(path)
    Kinematics(init_robot, robot, G.qlist(path(i), :))
    Simulate(robot, obss, false)
    pause(0.1)
end

disp("END_Plot")

% Functions


function [isCollision, check_start_idx] = CheckGoal(init_robot, robot, obss, check_start_idx)
    global q_goal;
    global epsilon;
    global G;

    isCollision = true;
    for i=check_start_idx:height(G.qlist)
        if vecnorm(q_goal - G.qlist(i, :)) <= epsilon
            isCollision = CollisionCheck(init_robot, robot, obss, G.qlist(i, :), q_goal);
            if (isCollision == false)
                AddEdge(q_goal);
                ConnectEdge(G.qlist(i, :), q_goal);
            end
        end
    end

    check_start_idx = height(G.qlist);
    
end

function Draw_Init_Goal(init_robot, robot, q_init, q_goal)
    Kinematics(init_robot, robot, q_init)
    [~, q_init_robot] = show(robot);
    q_init_robot.FaceColor = [0 1 0];
    q_init_robot.FaceAlpha = 1;
    
    hold on;

    Kinematics(init_robot, robot, q_goal)
    [~, q_goal_robot] = show(robot);
    q_goal_robot.FaceColor = [1 0 0];
    q_goal_robot.FaceAlpha = 1;
end

function Expansion_from_q_near(init_robot, robot, obss, q_near, q_rand)
    global G;
    global gamma;

    q_base = q_near;
    q_target = q_rand;

    if vecnorm(q_target - q_base) < gamma
       isCollision = CollisionCheck(init_robot, robot, obss, q_base, q_target);
       if isCollision == false
           AddEdge(q_target);
           ConnectEdge(q_base, q_target);
       end 
    else
        while(true)
            % gamma 내 범위로 도달한 경우
            if vecnorm(q_base - q_target) < gamma
%                 isCollision = CollisionCheck(init_robot, robot, obss, q_base, q_target);
%                 if isCollision == false
%                     AddEdge(q_target);
%                     ConnectEdge(q_base, q_target);
%                 end
                break;
            end
            
            % gamma까지 직진.
            q_new = Make_q_new(q_base, q_target);
            isCollision = CollisionCheck(init_robot, robot, obss, q_base, q_new);
            if isCollision == true
                break
            else
                AddEdge(q_new);
                ConnectEdge(q_base, q_new);
                q_base = q_new;
            end
        end
    end
end

function isCollision = CollisionCheck(init_robot, robot, obss, q_from, q_to)
    global G;
    isCollision = false;
    precision = 5;
    
    tmpVec = q_to - q_from;
    unit_tmpVec = tmpVec / precision;

    hold off;
    for i=1:precision
        Kinematics(init_robot, robot, q_from + unit_tmpVec * i);
        colNum = CheckCollision_Robot(robot, obss);

%         Simulate(robot, obss, true);
%         pause(0.1)

        if colNum ~= 0
            isCollision = true;
            break;
        end
    end
end

function q_sample = SamplingQ(init_robot, robot, obss)
    global q_goal;
    while(true)
        ep = randi(5);      % 1 ~ 8
        q_sample = q_goal;
        if ep <= 1
            break;  
        else
            % Map1
%             q_sample(1) = 0; %-5 + (5-(-5)) * rand();
%             q_sample(2) = -25 + (25-(-25)) * rand();
%             q_sample(3) = -1 + (10-(-1)) * rand();
%     
%             q_sample(4) = -100 + (100-(-100)) * rand();
%             q_sample(5) = -40 + (40-(-40)) * rand();
%             q_sample(6) = 0 + (0-(-0)) * rand();
%     
%             q_sample(4) = deg2rad(q_sample(4));
%             q_sample(5) = deg2rad(q_sample(5));
%             q_sample(6) = deg2rad(q_sample(6));

            % Map3
            q_sample(1) = -5 + (10-(-5)) * rand();
            q_sample(2) = -4 + (15-(-4)) * rand();
            q_sample(3) = -3 + (12-(-3)) * rand();
    
            q_sample(4) = -90 + (90-(-90)) * rand();
            q_sample(5) = -90 + (90-(-90)) * rand();
            q_sample(6) = -90 + (90-(-90)) * rand();
    
            q_sample(4) = deg2rad(q_sample(4));
            q_sample(5) = deg2rad(q_sample(5));
            q_sample(6) = deg2rad(q_sample(6));
        end

        Kinematics(init_robot, robot, q_sample);
        colNum = CheckCollision_Robot(robot, obss);
    
        if colNum == 0
            break
        end
    end
end

% 나중에 거리를 weight로 둬서 해보기.
function nodeMap = MakeGraph()
    global G;
    [rows, cols] = size(G.map);
    s = [];
    t = [];
    for i=1:rows
        for j=i+1:cols
            if G.map(i, j) == 1
                s = [s;i];
                t = [t;j];
            end
        end
    end
    
    nodeMap = graph(s, t);
    % plot(nodeMap)
end

function ConnectEdge(q1, q2)
    global G;
    q1_idx = Find_q_idx(q1);
    q2_idx = Find_q_idx(q2);

    if  q1_idx == -3 || q2_idx == -3
        disp("ll")
    end

    G.map(q1_idx, q2_idx) = 1;
    G.map(q2_idx, q1_idx) = 1;
end

% Base: q1, Direction: q2
function q_new = Make_q_new(q1, q2)
    global G;
    global gamma;
    diff = q2 - q1;
    dist = vecnorm(diff);

    unitVec = diff / dist;
    q_new = q1 + unitVec * gamma;

    % disp("q1:"+q1+ ", q2:"+q2+"\n")
end

function min_idx = Find_q_near(q)
    global G;
    [minval, min_idx] = min(vecnorm(G.qlist - q, 2, 2));
end

function q_idx = Find_q_idx(q)
    global G;
    q_idx = -3;
    for i=height(G.qlist):-1:1
        if G.qlist(i, :) == q
            q_idx = i;
            break
        end
    end
end

function AddEdge(q)
    global G;
    G.map = [G.map, zeros(1, length(G.map))'];
    G.map = [G.map; zeros(1, length(G.map))];
    G.qlist = [G.qlist; q];
end

function colNum = CheckCollision_Robot(robot, obss)
    colNum = 0;
    for i=1:length(obss)
        isCol = checkCollision(robot, obss(i));
        if isCol == true
            colNum = colNum + 1;
        end
    end
end

function Kinematics(init_robot, robot, q)
    H01 = TXYZ([q(1), q(2), q(3)]) * ROLL(q(4)) * PITCH(q(5)) * YAW(q(6));
    robot.Pose = H01 * init_robot.Pose;
end

function Simulate(robot, obss, collView)
    fig=gcf;
    fig.Position(1:4) = [100, 100, 1800, 600];

    if collView == true
        SimulRobot(robot);
        hold on
%         view(-78, 8)
        view(2)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);

    else
        subplot(1,3,1)
        SimulRobot(robot);
        hold on
        view(-90, 0)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    
        subplot(1,3,2)
        SimulRobot(robot);
        hold on
        view(-90, 90)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    
        subplot(1,3,3)
        SimulRobot(robot);
        hold on
        view(3)
    
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    end

end

function SimulRobot(robot)
    [~, q_robot] = show(robot);
    q_robot.FaceColor = [255/255, 192/255, 203/255];
    q_robot.FaceAlpha = 0.8;
    q_robot.EdgeColor = 'none';
end

function SimulObss(obss)
    for i=1:length(obss)
        [~, obs_paint] = show(obss(i));
        obs_paint.FaceAlpha = 0.1;
        obs_paint.FaceColor = [255/255, 255/255, 0/255];
        obs_paint.EdgeColor = 'none';
    end
end

function Ryaw = YAW(yaw)
    Ryaw = [
    1, 0, 0, 0;
    0, cos(yaw), -sin(yaw), 0;
    0, sin(yaw), cos(yaw), 0;
    0, 0, 0, 1];
end

function Rpitch = PITCH(pitch)
    Rpitch = [
    cos(pitch), 0, sin(pitch), 0;
    0, 1, 0, 0;
    -sin(pitch), 0, cos(pitch), 0;
    0, 0, 0, 1];
end

function Rroll = ROLL(roll)
    Rroll = [
    cos(roll), -sin(roll), 0, 0;
    sin(roll), cos(roll), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
end

function Txyz = TXYZ(txyz)
    tx = txyz(1);
    ty = txyz(2);
    tz = txyz(3);

    Txyz = [
    1, 0, 0, tx;
    0, 1, 0, ty;
    0, 0, 1, tz;
    0, 0, 0, 1];
end

