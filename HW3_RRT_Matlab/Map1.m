function obss=Map1()
    obs1 = collisionBox(20, 1, 13);          % Top
    obs2 = collisionBox(20, 1, 13);          % Bottom
    obs3 = collisionBox(10, 1, 20);          % Left
    obs4 = collisionBox(10, 1, 20);          % Right

    obs5 = collisionBox(20, 1, 13);          % Top 2
    obs6 = collisionBox(20, 1, 13);          % Bottom 2
    obs7 = collisionBox(10, 1, 20);          % Left 2
    obs8 = collisionBox(10, 1, 20);          % Right 2

    obs9 = collisionBox(20, 1, 13);          % Top 3
    obs10 = collisionBox(20, 1, 13);          % Bottom 3
    obs11 = collisionBox(10, 1, 20);          % Left 3
    obs12 = collisionBox(10, 1, 20);          % Right 3
    
    % 중심점 이동
    obs1.Pose = trvec2tform([0 -15 8.5]);
    obs2.Pose = trvec2tform([0 -15 -8.5]);
    obs3.Pose = trvec2tform([8.5 -15 0]);
    obs4.Pose = trvec2tform([-8.5 -15 0]);

    obs5.Pose = trvec2tform([0 0 12.5]);
    obs6.Pose = trvec2tform([0 0 -4.5]);
    obs7.Pose = trvec2tform([8.5 0 4]);
    obs8.Pose = trvec2tform([-8.5 0 4]);

    obs9.Pose = trvec2tform([0 15 8.5]);
    obs10.Pose = trvec2tform([0 15 -8.5]);
    obs11.Pose = trvec2tform([8.6 15 0]);
    obs12.Pose = trvec2tform([-8.6 15 0]);

    obss = [obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9, obs10, obs11, obs12];
end
