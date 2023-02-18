clear all; clc; close all;

[links, joints] = Robot2();
[init_links, init_joints] = Robot2();

% x, y, z, r1, p1, y1, r2, p2, y2
q = [0, 0, 0, deg2rad(30), deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)];
Kinematics(init_links, init_joints, links, joints, q);

SimulRobot(links);
SimulRobot(joints);

function SimulObss(obss)
    for i=1:length(obss)
        [~, obs_paint] = show(obss(i));

        hold on;
        %view(-74, 2)
        xlim([-10 25])
        ylim([-10 25])
        zlim([-10 25])

        obs_paint.FaceAlpha = 0.1;
        obs_paint.FaceColor = [255/255, 255/255, 0/255];
        obs_paint.EdgeColor = 'none';
    end
end

function SimulRobot(parts)
    for i=1:length(parts)
        [~, q_part] = show(parts(i));

        hold on;
        %view(-74, 2)
        xlim([-10 25])
        ylim([-10 25])
        zlim([-10 25])

        q_part.FaceColor = [255/255, 192/255, 203/255];
        q_part.FaceAlpha = 1;
        q_part.EdgeColor = 'none';
    end
end

function Kinematics(init_links, init_joints, links, joints, q)
    H01 = TXYZ([q(1), q(2), q(3)]) * ROLL(q(4)) * PITCH(q(5)) * YAW(q(6));
    H12 = ROLL(q(7)) * PITCH(q(8)) * YAW(q(9));

    extra_link_length = [1 0 0 1.5;
                       0 1 0 0;
                       0 0 1 0;
                       0 0 0 1];
    joints(1).Pose = H01 * init_joints(1).Pose;
    joints(2).Pose = joints(1).Pose + H01 * H12 * joints(2).Pose;

    links(1).Pose = joints(1).Pose * extra_link_length;
    links(2).Pose = joints(2).Pose * extra_link_length;

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
