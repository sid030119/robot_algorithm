function obss=Map0()
    obs1 = collisionBox(20, 1, 13);          % Top
    obs2 = collisionBox(20, 1, 13);          % Bottom
    obs3 = collisionBox(10, 1, 20);          % Left
    obs4 = collisionBox(10, 1, 20);          % Right
    
    % 중심점 이동
    obs1.Pose = trvec2tform([0 0 8.5]);
    obs2.Pose = trvec2tform([0 0 -8.5]);
    obs3.Pose = trvec2tform([8.5 0 0]);
    obs4.Pose = trvec2tform([-8.5 0 0]);

    obss = [obs1, obs2, obs3, obs4];
end
