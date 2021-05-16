classdef Leg < handle
    %leg 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        base_pe
        base_pm
        U1ipe
        U2ipe
        U3ipe
        S2ipe
        S3ipe
        Sfipe
        Sfipe_tracker %靶球在脚踝坐标系下的坐标，标定用
        home_pos
        Pee_b %身体坐标系下
        Pee %腿坐标系下
        Pee_tracker %标定用，靶球坐标
        Pin %各支链长度
        q %关节变量，减去home长度
        Fee_b %足尖力，在身体坐标系下
        Fee_l %足尖力，在腿坐标系下
        Fin %驱动关节的推力
        Kq %驱动支链刚度
        alpha %主支链万向节第一转角，绕Y轴
        beta %主支链万向节第二转角，绕局部Z轴
        stroke %行程
        jac
    end
    
    methods
        function obj = Leg(config,stroke,base_pe,Sfipe_tracker)
            %leg 构造此类的实例
            %   此处显示详细说明
            if nargin < 4
                Sfipe_tracker = zeros(3,1);
                if nargin < 3
                    base_pe = zeros(1,6);
                end
            end
            u2y = config(1);
            u2z = config(2);
            u3y = config(3);
            u3z = config(4);
            s2x = config(5);
            s2y = config(6);
            s2z = config(7);
            s3x = config(8);
            s3y = config(9);
            s3z = config(10);
            sfx = config(11);
            sfy = config(12);
            sfz = config(13);
%             gamma = config(10);
            home_pos = config(14:16);
            
%             Sfi = RotX(gamma)*[sfx; sfy; sfz];
%             S2i = RotX(gamma)*[0; sy; sz];
%             S3i = RotX(gamma)*[0; sy; -sz];
            Sfi = [sfx; sfy; sfz];
            S2i = [s2x; s2y; s2z];
            S3i = [s3x; s3y; s3z];
            
            obj.base_pe = base_pe;
            obj.base_pm = pe2pm(obj.base_pe);
            obj.U1ipe = [0 0 0 pi/2 pi/2 0]';
            obj.U2ipe = [0 u2y u2z pi/2 pi/2 0]';
            obj.U3ipe = [0 u3y u3z pi/2 pi/2 0]';
            obj.Sfipe = [Sfi; 0; 0; 0];
            obj.S2ipe = [S2i; 0; 0; 0];
            obj.S3ipe = [S3i; 0; 0; 0];
            obj.home_pos = home_pos(:);
            obj.stroke = stroke;
            obj.Sfipe_tracker = Sfipe_tracker;
        end
        
        function setPee(obj,pee,Marker)
            %setPee 设置末端位置
            %   此处显示详细说明
            if nargin < 3
                Marker = 'L';
            end
            switch Marker
                case 'L'
                    % 把足尖坐标从腿坐标系变换到身体坐标系
                    obj.Pee = pee(:);
                    Pee2b = obj.base_pm*[obj.Pee;1];
                    obj.Pee_b = Pee2b(1:3);
                case 'B'
                    % 把足尖坐标从身体坐标系变换到腿坐标系
                    obj.Pee_b = pee(:);
                    Pee2l = obj.base_pm\[obj.Pee_b;1];
                    obj.Pee = Pee2l(1:3);
                otherwise
                    error('Marker input error');
            end
            obj.inverseKinematics();
        end
        
        function inverseKinematics(obj)
            %inverseKinematics 运动学反解
            %   在腿坐标系下计算
            
            x = obj.Pee(1);
            y = obj.Pee(2);
            z = obj.Pee(3);
            
            Sfx = obj.Sfipe(1);
            Sfy = obj.Sfipe(2);
            Sfz = obj.Sfipe(3);
            l1 = sqrt(x.^2 + y.^2 + z.^2 - Sfy.^2 - Sfz.^2) - Sfx;
            beta1 = asin(y ./ sqrt((l1 + Sfx).^2 + Sfy.^2)) - asin(Sfy ./ sqrt((l1 + Sfx).^2 + Sfy.^2));
            tmp = (l1 + Sfx) * cos(beta1) - Sfy * sin(beta1);
            alpha1 = atan((Sfz * x - tmp * z) / (tmp * x + Sfz * z));
            rotm = RotY(alpha1) * RotZ(beta1);
            TR = eye(4);
            TR(1:3,1:3) = rotm;
            TP = eye(4);
            TP(1,4) = l1;
            T_l2f = TR * TP;
            S2_f = [obj.S2ipe(1:3); 1];
            S3_f = [obj.S3ipe(1:3); 1];
            S2_l = T_l2f * S2_f;
            S3_l = T_l2f * S3_f;
            S2 = S2_l(1:3);
            S3 = S3_l(1:3);
            U2 = obj.U2ipe(1:3);
            U3 = obj.U3ipe(1:3);
            V2 = S2 - U2;
            V3 = S3 - U3;
            l2 = norm(V2);
            l3 = norm(V3);
            alpha2 = atan(-V2(3) / V2(1));
            alpha3 = atan(-V3(3) / V3(1));
            beta2 = asin(V2(2) / norm(V2));
            beta3 = asin(V3(2) / norm(V3));
            
            obj.Pin = [l1 l2 l3]';
            obj.alpha = [alpha1 alpha2 alpha3]';
            obj.beta = [beta1 beta2 beta3]';
            obj.q = obj.Pin - obj.home_pos;
            %计算靶球坐标
            Sf_tracker_l = T_l2f * [obj.Sfipe_tracker;1];
            obj.Pee_tracker = Sf_tracker_l(1:3);
        end
        
        function calcVelocityJacobian(obj)
            %calVelJac 计算速度雅可比矩阵
            %   在腿坐标系下计算
            x = obj.Pee(1);
            y = obj.Pee(2);
            z = obj.Pee(3);
            U2 = obj.U2ipe(1:3);
            U3 = obj.U3ipe(1:3);
            S2 = obj.S2ipe(1:3);
            S3 = obj.S3ipe(1:3);
%             Sfx = obj.Sfipe(1);
%             Sfy = obj.Sfipe(2);
%             Sfz = obj.Sfipe(3);
            l1 = obj.Pin(1);
            l2 = obj.Pin(2);
            l3 = obj.Pin(3);
            alpha1 = obj.alpha(1);
            beta1 = obj.beta(1);
			sa1 = sin(alpha1);
			sb1 = sin(beta1);
			ca1 = cos(alpha1);
			cb1 = cos(beta1);
            
            Rk1 = [-sa1*cb1 sa1*sb1  ca1;
                          0       0    0;
                   -ca1*cb1 ca1*sb1 -sa1];
            Rk2 = [-ca1*sb1 -ca1*cb1 0;
                        cb1     -sb1 0;
                    sa1*sb1  sa1*cb1 0];
                
            k21 = U2'*Rk1*[S2(1)+l1; S2(2); S2(3)];
            k22 = U2'*Rk2*[S2(1)+l1; S2(2); S2(3)];
            k23 = U2'*[ca1*cb1; sb1; -sa1*cb1] - l1 - S2(1);
            
            k31 = U3'*Rk1*[S3(1)+l1; S3(2); S3(3)];
            k32 = U3'*Rk2*[S3(1)+l1; S3(2); S3(3)];
            k33 = U3'*[ca1*cb1; sb1; -sa1*cb1] - l1 - S3(1);
            
            J1 = [ z,      -y*ca1,  ca1*cb1;
                   0, x*ca1-z*sa1,      sb1;
                  -x,       y*sa1, -sa1*cb1];
            J2 = [       0       0       1
                  -k21/l2 -k22/l2 -k23/l2;
                  -k31/l3 -k32/l3 -k33/l3];
            obj.jac = J1/J2;
        end
        
        function Fin = calcJointForces(obj, Fee)
            %calcJointForces 由足尖力计算关节力
            %   Fee为足尖力，是三维向量，在腿坐标系下表示
            Fee = Fee(:);
            obj.calcVelocityJacobian();
            Fin = obj.jac'*Fee;
        end
        
        function Fee2b = calcFee(obj, Fin)
            %calcFee 由关节力计算足尖力
            %   Fee为足尖力，是三维向量，在腿坐标系下表示
            Fin = Fin(:);
            obj.calcVelocityJacobian();
            Fee2l = obj.jac'\Fin;%三维力矢量，腿坐标系下
            Fee2b = obj.base_pm(1:3,1:3)*Fee2l;%变换到身体坐标系下
        end
        
        function forwardKinematics(obj, pin, init_pee)
            %forwardKinematics 运动学正解
            %   pin是关节坐标
            %   init_pee是初始足尖坐标，三维向量，在腿坐标系下表达
            %   计算后更新Pee属性
            if nargin == 2
                init_pee = obj.Sfipe(1:3) + [obj.home_pos(1);0;0];
            end
            pin = pin(:);
            X = init_pee(:);
            dq = ones(3,1);
            eps = 10e-9;
            n = 0;
            while norm(dq) > eps
                X0 = X;
                obj.setPee(X0);
                q0 = obj.Pin;
                obj.calcVelocityJacobian();
                X = X0 + obj.jac * (pin - q0);
                dq = pin - q0;
                n = n + 1;
                %disp(dq)%%测试用
            end
            %disp(n);%%测试用
            obj.setPee(X);
        end
        
        function setQ(obj, q, init_pee, Marker)
            %setQ 设置输入变量
            %   此处显示详细说明
            if nargin < 4
                Marker = 'L';
                if nargin < 3
                    init_pee = obj.Sfipe(1:3) + [obj.home_pos(1);0;0];
                end
            end
            switch Marker
                case 'L'
                    % 把足尖坐标从腿坐标系变换到身体坐标系
                    init_pee2l = init_pee(:);
                case 'B'
                    % 把足尖坐标从身体坐标系变换到腿坐标系
                    init_pee2l = obj.base_pm\[init_pee(:);1];
                    init_pee2l = init_pee2l(1:3);
                otherwise
                    error('Marker input error');
            end
            
            obj.q = q(:);
            pin = q(:) + obj.home_pos;
            obj.forwardKinematics(pin, init_pee2l);
        end

        function bool_out = isInWorkspace(obj)
            %isInWorkspace 判断当前末端位置是否在工作空间内
            %   此处显示详细说明
            bool_out = true;
            for i = 1:3
                if obj.q(i) < 0 || obj.q(i) > obj.stroke
                    bool_out = false;
                    break;
                end
            end
            % U副转角限制
            if max(abs(obj.alpha)) > 45/180*pi
                bool_out = false;
            end
            if max(abs(obj.beta)) > 45/180*pi
                bool_out = false;
            end
%             if cos(obj.alpha(1))*cos(obj.beta(1)) < cos(45/180*pi)
            if min(cos(obj.alpha).*cos(obj.beta)) < cos(45/180*pi)
                bool_out = false;
            end
        end
        
        function calcJointStiffness(obj)
            %calcJointStiffness 计算关节刚度
            %   此处显示详细说明
            
        end

        function plotWorkspace(obj)
            %plotWorkspace 绘制工作空间示意图
            %   此处显示详细说明
            alpha = -90:1:90;
            beta = -90:1:90;
            dr = obj.stroke/100;
            r = obj.home_pos(1);
        end
    end
end

