% 产生仿真使用的角度值
function [angle] = test_angle(xb06,sample)
angle = zeros(sample,4,6);
jnt_err = [0.002,0.002,0.002,0.002,0.002,0.002];
side_x = [0.05;0;0];
side_y = [0;0.05;0];
side_xy = [0.05;0.05;0];
fk1 = zeros(4,4);
fk2 = zeros(4,4);
fk3 = zeros(4,4);
fk4 = zeros(4,4);

for i = 1:sample
    init_pos = randn(3,1);
    rot = randn(3,1) / 3;
    fk1(1:3,4) = init_pos;
    fk1(1:3,1:3) = rotz(rot(1)) * roty(rot(2)) * rotx(rot(3));
    fk1(4,:) = [0,0,0,0];
    jnt_pos = xb06.ikine(fk1);
    angle(i,1,:) = jnt_pos - jnt_err;
    
    fk2(1:3,4) = init_pos + side_x;
    fk2(1:3,1:3) = fk1(1:3,1:3);
    fk2(4,:) = [0,0,0,0];
    jnt_pos = xb06.ikine(fk2);
    angle(i,2,:) = jnt_pos - jnt_err;
    
    fk3(1:3,4) = init_pos + side_xy;
    fk3(1:3,1:3) = fk1(1:3,1:3);
    fk3(4,:) = [0,0,0,0];
    jnt_pos = xb06.ikine(fk3);
    angle(i,3,:) = jnt_pos - jnt_err;
    
    fk4(1:3,4) = init_pos + side_y;
    fk4(1:3,1:3) = fk1(1:3,1:3);
    fk4(4,:) = [0,0,0,0];
    jnt_pos = xb06.ikine(fk4);
    angle(i,4,:) = jnt_pos - jnt_err;
end