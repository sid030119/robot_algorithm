function obss=Map2()
    obs1 = collisionBox(10, 4, 0.1);
    obs2 = collisionBox(10, 0.1, 4);
    obs3 = collisionBox(10, 4, 0.1);
    obs4 = collisionBox(6, 0.1, 4);
    obs5 = collisionBox(0.1, 10, 4);
    obs6 = collisionBox(4, 6, 0.1);
    obs7 = collisionBox(4, 6, 0.1);
    obs8 = collisionBox(0.1, 6, 4);
   
    % 중심점 이동
    obs1.Pose = trvec2tform([0 0 2]);
    obs2.Pose = trvec2tform([0 -2 0]);
    obs3.Pose = trvec2tform([0 0 -2]);
    obs4.Pose = trvec2tform([-2 2 0]);
    obs5.Pose = trvec2tform([5 3 0]);
    obs6.Pose = trvec2tform([3 5 2]);
    obs7.Pose = trvec2tform([3 5 -2]);
    obs8.Pose = trvec2tform([1 5 0]);
    

    obss = [obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8];
end
