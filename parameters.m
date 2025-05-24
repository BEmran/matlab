% Quadrotor's Parameters
% motor coffecients
QP.ThrustCoff= 0.01; % thrust coffecient [N.s^2/rad^2]
QP.DragRatio = 0.1; % drag to thrust ratio [1/m]
QP.ArmLength = 0.2; % arm length [m]

QP.Gu = [            1,             1,            1,             1;...
         -QP.ArmLength,             0, QP.ArmLength,             0;...
                     0,  QP.ArmLength,            0, -QP.ArmLength;...
          QP.DragRatio, -QP.DragRatio, QP.DragRatio, -QP.DragRatio] *  QP.ThrustCoff;

% Omegas min/max value [rad/sec]
QP.WMax = 70;
QP.WMin = 30;

% Tarque min/max values [N.m] 
QP.TauPQMax = 0.1;
QP.TauPQMin = -0.1;
QP.TauRMax = 0.05;
QP.TauRMin = -0.05;

% inertia Kg/m
QP.Ixx = 1.1;
QP.Iyy = 1.5;
QP.Izz = 1.3;
QP.Ixyz = 0.8;
QP.InertiaMatrix = diag([QP.Ixx, QP.Iyy, QP.Izz]) + ones(3) * QP.Ixyz;

QP.mass = 1.2; % UAV mass [Kg]

% acceleration = theta * u
tmp = zeros(4,4);
tmp(1) = 1/QP.mass;
tmp(2:4, 2:4) = inv(QP.InertiaMatrix);
QP.theta = tmp * QP.Gu;

% Initial Condition
QP.InertiaMatrixMax = ones(3) * 2.5;
QP.InertiaMatrixMin = ones(3) * 0.5;
QP.InertiaMatrixGamma = 0.1;
QP.InertiaMatrix0 = diag([QP.Ixx, QP.Iyy, QP.Izz]) * 0.9 + QP.InertiaMatrixMin;
