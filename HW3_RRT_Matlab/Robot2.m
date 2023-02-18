function [links, joints] = Robot2()
    link = [3 1 1];
    link1 = collisionBox(link(1), link(2), link(3));
    link2 = collisionBox(link(1), link(2), link(3));

    joint1 = collisionSphere(0.5);
    joint2 = collisionSphere(0.5);
    

    link1.Pose = trvec2tform([1.5 0 0]);
    link2.Pose = trvec2tform([4.5 0 0]);

    joint1.Pose = trvec2tform([0 0 0]);
    joint2.Pose = trvec2tform([3 0 0]);

    links = [link1, link2];
    joints = [joint1, joint2];
end