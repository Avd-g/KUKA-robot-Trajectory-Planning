
    close all;
    clear all;

    % Constants
    mm = 1/1000;
    steps = 100000;
    Ti = eye(4);
    qc = [97.31 -4.70 162.57 103.53 -1.96 -62.33 113.31];
    a = zeros(7);
    d = [0.34, 0, 0.4, 0, 0.4, 0, 0.126];
    alpha = [-pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2, 0];
    yef = 0.033+(0.025/2);
    zef = 0.0600;
    xef = 0.00;

    % Transformation matrices
    T2 = [0, 1, 0, 0; -1, 0, 0, -0.0662; 0, 0, 1, 0.0431; 0, 0, 0, 1];
    roll_x = 174.1750404305652;
    pitch_y = -17.3967534123935;
    yaw_z = -1.9587388578232;
    angles = [yaw_z pitch_y roll_x];
    rad = deg2rad(angles);
    rot_ac = eul2rotm(rad, 'ZYX');
    T3 = zeros(4);
    T3(1:3, 1:3) = rot_ac;
    T3(4,:) = [0, 0, 0, 1];
    T3(:, 4) = [-0.205780720039398, -0.109793029482687, 0.561252115509121, 1];

    T4 = [1, 0, 0, 0.103975 ; 0, 1, 0, -0.103975 ; 0, 0, 1, 0 ; 0, 0, 0, 1];
    Tf = T2 * T3 * T4;

    % Compute A matrices
    A = compute_A_matrices(qc, a, d, alpha);

    % Compute Tbt
    Tbt = compute_Tbt(Tf, A);

    % Compute Tef
    Tef = compute_Tef(xef, yef, zef);

    % Compute Tbe
    Tbe = Tbt * inv(Tef);

    d_pose = pose(Tbe);

    % Display result
    disp('Tbe:');
    disp(Tbe);
    % Extracting pose
pose_Tbe = pose(Tbe);

% Extracting phi, theta, psi from pose
phi = pose_Tbe(4);
theta = pose_Tbe(5);
psi = pose_Tbe(6);
Ta = [1 0 0 0 0 0; 
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 -sin(phi) cos(phi)*sin(theta);
      0 0 0 0 cos(phi) sin(phi)*sin(theta);
      0 0 0 1 0 cos(theta)];
% Pre-allocate Q for efficiency
Q = zeros(7, steps);
Q(:,1) = deg2rad([58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712]);
K = 10*eye(6,6);
e = zeros(6,steps);

% Define joint limits as column vectors
q_min = deg2rad([-170; -120; -170; -120; -170; -120; -175]);
q_max = deg2rad([170; 120; 170; 120; 170; 120; 175]);

for i = 1:steps 
    % Forward kinematics
    [T02e, A02e] = FK(d, transpose(Q(:,i)), a, alpha, Ti); 
    xe = pose(T02e);
    e(:,i) = pose_Tbe - xe;
    Ja = Jg2Ja(A02e,Ta);
    qdot = pinv(Ja)*K*e(:,i);
    
    % Update joint positions
    for j = 1:7
        if Q(j,i) >= q_min(j) && Q(j,i) <= q_max(j)
            Q(j,i+1) = Q(j,i) + qdot(j)*0.01; 
        else
            Q(j,i+1) = q_min(j);
        end
    end
    
    % Check for convergence
    if max(abs(e(:,i))) < 0.0001
        qfinal = Q(:,i); 
        break;
    end
end

qfinal = Q(:, i);
    

[T0des, A0des] = FK(d, transpose(qfinal), a, alpha, Ti);
qfin_deg =  rad2deg(qfinal(:,1));
a0 = deg2rad([58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712 ]);
a1 = 0;
a2 = 0;
A = [10^3, 10^4, 10^5; 3*10^2, 4*10^3, 5*10^4; 6*10, 12*10^2, 20*10^3];
x = zeros(3,7);
b = zeros(3,7);

for i = 1:7
    b(:,i) = [qfinal(i,1) - a0(1,i); 0; 0];
    x(:,i) = A\b(:,i);
end
Tinitial = 0;
Tfinalt = 10;
t = transpose(linspace(Tinitial, Tfinalt, 2000));
lt = length(t);

PoseG = zeros(7, lt);
VeloG = zeros(7, lt);

for i = 1:lt
    t_i = t(i);
    t_i_sq = t_i^2;
    t_i_cube = t_i^3;
    t_i_quad = t_i^4;
    t_i_quint = t_i^5;
    
    for j = 1:7
        PoseG(j, i) = x(3, j) * t_i_quint + x(2, j) * t_i_quad + x(1, j) * t_i_cube + a2 * t_i_sq + a1 * t_i + a0(j); % Joint trajectory
        VeloG(j, i) = 5 * x(3, j) * t_i_quad + 4 * x(2, j) * t_i_cube + 3 * x(1, j) * t_i_sq + 2 * a2 * t_i + a1; % Velocity Profile
    end
end

Qgraph = PoseG; % Assigning PoseG to Qgraph directly
Qdgraph = rad2deg(VeloG); % Converting velocity to degrees and assigning to Qdgraph

writematrix(round(Qgraph,4)','Gaur_Avdhesh_file.txt','Delimiter','space')
subplot(3,1,1);
 x = t;
 y1 = PoseG;
 plot(x,y1)
 title('Joint Trajectory')
 
  subplot(3,1,2); 
 y2 = VeloG;
  plot(x,y2)
 title('Velocity Profile')
function A = compute_A_matrices(qc, a, d, alpha)
    the = deg2rad(qc);
    A = zeros(4, 4, 7);
    for i = 1:7
        A(:,:,i) = [cos(the(i)), -sin(the(i))*cos(alpha(i)), sin(the(i))*sin(alpha(i)), a(i)*cos(the(i));
                    sin(the(i)), cos(the(i))*cos(alpha(i)), -cos(the(i))*sin(alpha(i)), a(i)*sin(the(i));
                    0, sin(alpha(i)), cos(alpha(i)), d(i);
                    0, 0, 0, 1];
    end
end

function Tbt = compute_Tbt(Tf, A)
    A17 = eye(4);
    for i = 1:7
        A17 = A17 * A(:, :, i);
    end
    Tbt = A17 * Tf;
end

function Tef = compute_Tef(xef, yef, zef)
    Tef = eye(4);
    ef_rot_z = myrotmat(-pi/2,'z');
    ef_rot_x = myrotmat(pi,'x');
    ef_new_rot = ef_rot_z * ef_rot_x;
    Tef(1:3, 1:3) = ef_new_rot;
    Tef(4, :) = [0 0 0 0];
    Tef(:, 4) = [xef yef zef 1];
end

function R = myrotmat(theta, axis)
    switch axis
        case {'x','X'}
            R = [1 0 0 ; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
        case {'y','Y'}
            R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        case {'z','Z'}
            R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
        otherwise
            disp('Unknown axis. Please use x, y or z');
            R = [];
    end
end
function ps = pose(Td)
    pd = Td(1:3, 4);
    phi = atan2(Td(2, 3), Td(1, 3));
    theta = atan2(sqrt(Td(1, 3)^2 + Td(2, 3)^2), Td(3, 3));
    psi = atan2(Td(3, 2), -Td(3, 1));
    phid = [phi; theta; psi];
    ps = [pd; phid];
end
function Ja = Jg2Ja(Ac,Ta)
    %Performs Jacobian Here from Transformations
    z0 = [0; 0; 1]; 
    p0 = [0; 0; 0];
    pe = Ac{1,7}(1:3,3);
    
    z1 = Ac{1,1}(1:3,3); 
    p1 = Ac{1,1}(1:3,4);
    
    z2 = Ac{1,2}(1:3,3);
    p2 = Ac{1,2}(1:3,4);
    
    z3 = Ac{1,3}(1:3,3); 
    p3 = Ac{1,3}(1:3,4);
    
    z4 = Ac{1,4}(1:3,3); 
    p4 = Ac{1,4}(1:3,4);
    
    z5 = Ac{1,5}(1:3,3); 
    p5 = Ac{1,5}(1:3,4);
    
    z6 = Ac{1,6}(1:3,3); 
    p6 = Ac{1,6}(1:3,4);
    
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
        z0 z1 z2 z3 z4 z5 z6];
    
    Ja = inv(Ta)*Jg; 
end
function [T, Tloop] = FK(d, q, a, alpha, Tin)

  T = Tin;
  R = length(a);
  Tloop = cell(1, R);

  % Pre-calculate sine and cosine for efficiency
  sq = sin(q);
  cq = cos(q);
  sa = sin(alpha);
  ca = cos(alpha);

  for j = 1:R
    % Construct homogeneous transformation matrix using pre-calculated values
    T = T * [cq(j), -sq(j)*ca(j), sq(j)*sa(j), a(j)*cq(j);
              sq(j),  cq(j)*ca(j), -cq(j)*sa(j), a(j)*sq(j);
                 0,             sa(j),             ca(j),  d(j);
                 0,                 0,                 0,  1];
    Tloop{j} = T;
  end
end
