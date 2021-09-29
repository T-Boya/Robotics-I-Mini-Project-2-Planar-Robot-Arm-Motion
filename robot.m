classdef robot < handle
    properties
        q % n × 1 joint displacement vector
        P % 3 × n + 1 link displacement vector in the zero configuration
        T % 4 × 4 homogeneous transform for the pose (orientation and position) of the robot
        J % 3×n matrix relating joint velocity to the end effector angular and linear velocities
        qsol % q calculated by inverse kinematics
        normals
    end
    methods
        function obj = robot()
        end
        
        function forwards(obj)
            q = obj.q;
            % Rotation Matrices
            R01 = [cos(q(1)) -sin(q(1)); sin(q(1)) cos(q(1))];
            R02 = [cos(q(1) + q(2)) -sin(q(1) + q(2)); sin(q(1) + q(2)) cos(q(1) + q(2))];

            % Direction Vectors
            P01 = [0; 0];
            P12 = [1.5; 0];
            P23 = [1.5; 0];
            P3T = [0.5; 0];

            % Forward Kinematics
            R0T = [cos(sum(q)) -sin(sum(q)); sin(sum(q)) cos(sum(q))]; % end rotation matrix
            P0T = P01 + (R01 * P12) + (R02 * P23) + (R0T * P3T); % end translation
            obj.T = [R0T [0 0]' P0T; 0 0 1 0; 0 0 0 1]; % (ET, OT) homogeneous transform
            obj.J = [1 1 1;...
                -1.5*sin(q(1) + q(2))-1.5*sin(q(1))-0.5*sin(sum(q)) -1.5*sin(q(1)+q(2))-0.5*sin(sum(q)) -0.5*sin(sum(q));...
                1.5*cos(q(1) + q(2))+1.5*cos(q(1))+0.5*cos(sum(q)) 1.5*cos(q(1)+q(2))+0.5*cos(sum(q)) 0.5*cos(sum(q))];
        end
        
        function reverse(obj)
            T = obj.T;
            PT = [T(1:2,4)];
            % find q3 angle
            qT = atan2(T(2, 1), T(1, 1)) - pi;
            % Solve intersection of two circles representing links 1 and 2
            syms x y;
            P3 = [PT(1)-0.5*cos(qT) PT(2)-0.5*sin(qT)];
            eqn1 = (x - 0)^2 + (y - 0)^2 == 1.5^2;
            eqn2 = (x - P3(1))^2 + (y - P3(2))^2 == 1.5^2;
            [x, y] = solve([eqn1, eqn2], [x, y]);
            x = eval(x);
            y = eval(y);
            % calculate angles of P0 to intersection and P3 to
            % intersection relative to horizontal
            w1_1 = P3(1) - x(1);
            w1_2 = P3(1) - x(2);
            w2_1 = P3(2) - x(1);
            w2_2 = P3(2) - x(2);
            v1_1 = x(1);
            v1_2 = x(2);
            v2_1 = y(1);
            v2_2 = y(2);
            u1 = 1;
            u2 = 0;
            q1 = [atan2(u2*v1_1 - u1*v2_1, u1*v1_1+u2*v2_1), atan2(u2*v1_2 - u1*v2_2, u1*v1_2+u2*v2_2)];
            q2 = [atan2(w2_1*v1_1 - w1_1*v2_1, w1_1*v1_1+w2_1*v2_1), atan2(w2_2*v1_2 - w1_2*v2_2, w1_2*v1_2+w2_2*v2_2)];
            q3 = [qT-q1(1)-q2(1) qT-q1(2)-q2(2)];
            % q2 = [atan2(P3(2) - y(2), P3(1) - x(2)), atan2(P3(2) - y(1), P3(1) - x(1))];
            % create q vector
            obj.qsol = [q1(1) q2(1) q3(1); q1(2) q2(2) q3(2);];  
        end
        
        function compare(obj)
            obj.q
            obj.qsol
        end
    end
end