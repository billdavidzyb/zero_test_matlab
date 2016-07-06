%% zero and tcp calibration
% establish robot model
sample = 10;

L12z = 380;
L12x = 30;  
L23z = 340; 
L34z = 35; 
L34x = 120; 
L45x = 225;
L56x = 90;

d_n = zeros(1,6);
a_n = zeros(1,6);
p_n = zeros(1,6);
theta_e = zeros(1,6);

d_n(1) = L12z/1000; %link offset                         0.38
a_n(1) = L12x/1000; %link length                         0.03
a_n(2) = L23z/1000; %                                    0.34
a_n(3) = -L34z/1000; %           -0.035
d_n(4) = (L34x+L45x)/1000; %   0.345
d_n(6) = L56x/1000; %                               0.09

p_n(1) = -pi/2;
d_n(2) = 0;
p_n(2) = 0;
d_n(3) = 0;
p_n(3) = pi/2;
a_n(4) = 0;
p_n(4) = -pi/2;
d_n(5) = 0;
a_n(5) = 0;
p_n(5) = pi/2;
a_n(6) = 0;
p_n(6) = 0;

L_n(1) = Link('d', d_n(1), 'a', a_n(1), 'alpha', p_n(1));
L_n(2) = Link('d', d_n(2), 'a', a_n(2), 'alpha', p_n(2));
L_n(3) = Link('d', d_n(3), 'a', a_n(3), 'alpha', p_n(3));
L_n(4) = Link('d', d_n(4), 'a', a_n(4), 'alpha', p_n(4));
L_n(5) = Link('d', d_n(5), 'a', a_n(5), 'alpha', p_n(5));
L_n(6) = Link('d', d_n(6), 'a', a_n(6), 'alpha', p_n(6));
L_n(7) = Link('d', 0.1,    'a', 0,      'alpha', 0     );
xb06_n = SerialLink(L_n,'name', 'XB06_t', 'manufacturer', 'Rokae');   

coord = zeros(3,3);
jac_mat = zeros(3,3,7);
b = zeros(sample,1);
A1 = zeros(sample,7);
A = zeros(sample,5);
% acquire angle data
[angle] = test_angle(xb06_n,sample);    %sample * 3 * 7

for i = 1:sample
    for j = 1:3
        tmp_frm = xb06_n.fkine(angle(i,j,:));
        coord(:,j) = tmp_frm(1:3,4);
        tmp_jac = xb06_n.jacobn(angle(i,j,:));
        jac_mat(j,1:3,1:7) = tmp_jac(4:6,1:7);  %%%%%%%%%%%%%%%%%%%%%%
    end
    a1 = coord(1,1) - coord(1,2);
    a2 = coord(1,3) - coord(1,2);
    c1 = coord(2,1) - coord(2,2);
    c2 = coord(2,3) - coord(2,2);
    e1 = coord(3,1) - coord(3,2);
    e2 = coord(3,3) - coord(3,2);
    B1 = jac_mat(1,1,:) - jac_mat(2,1,:);
    B2 = jac_mat(3,1,:) - jac_mat(2,1,:);
    D1 = jac_mat(1,2,:) - jac_mat(2,2,:);
    D2 = jac_mat(3,2,:) - jac_mat(2,2,:);
    F1 = jac_mat(1,3,:) - jac_mat(2,3,:);
    F2 = jac_mat(3,3,:) - jac_mat(2,3,:);
    b(i) = - (a1 * a2 + c1 * c2 + e1 * e2);
    A1(i,:) = a1 * B2 + a2 * B1 + c1 * D2 + c2 * D1 + e1 * F2 + e2 * F1;
end
A = A1(:,1:5);
[Q,R] = qr(A);
result = pinv(R)*Q'*b


