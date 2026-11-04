% RoboControl 3D - Octave / MATLAB Kinematics Example
% This script validates the Denavit-Hartenberg transformations.

function kinematics()
    % DH Parameters: [a, alpha, d, theta_offset]
    % 3-DOF Arm configuration
    a = [0, 30, 25];
    alpha = [pi/2, 0, 0];
    d = [15, 0, 0];
    theta_offset = [0, 0, 0];

    % Test joint angles in degrees
    joint_angles = [45, -30, 20];
    
    % Convert to radians
    theta = deg2rad(joint_angles) + theta_offset;

    % Initial Transformation Matrix (Identity)
    T = eye(4);

    % Compute Forward Kinematics
    for i = 1:length(a)
        ct = cos(theta(i));
        st = sin(theta(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));

        % Standard DH Matrix
        A = [ct, -st*ca,  st*sa, a(i)*ct;
             st,  ct*ca, -ct*sa, a(i)*st;
              0,     sa,     ca,    d(i);
              0,      0,      0,       1];

        T = T * A;
    end

    % Extract End-Effector Position
    x = T(1, 4);
    y = T(2, 4);
    z = T(3, 4);

    fprintf('--- RoboControl 3D: Octave/MATLAB Forward Kinematics ---\n');
    fprintf('Joint Angles: [%d, %d, %d] deg\n', joint_angles(1), joint_angles(2), joint_angles(3));
    fprintf('End-Effector Position:\n');
    fprintf('X: %.2f cm\n', x);
    fprintf('Y: %.2f cm\n', y);
    fprintf('Z: %.2f cm\n', z);
end