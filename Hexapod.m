classdef Hexapod < handle
    %Hexapod 六足机器人
    %   此处显示详细说明
    
    properties
        peb
        pmb
        pee
        pee2b
        pin
        q
        leg_base_pe
        leg
        inv_jac %雅可比的逆
    end
    
    methods
        function obj = Hexapod(leg_config,stroke,leg_base_pe)
            %hexapod 构造此类的实例
            %   此处显示详细说明
            if nargin < 3
                obj.leg_base_pe = [ -0.43322 , 0 , -0.19907,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                                    -0.48305 , 0 ,  0,         pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                                    -0.43322 , 0 ,  0.19907,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                                    0.43322 , 0 , -0.19907,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                                    0.48305 , 0 ,  0,         pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                                    0.43322 , 0 ,  0.19907,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
            else
                obj.leg_base_pe = leg_base_pe;
            end
            obj.peb = zeros(1,6);
            obj.pmb = eye(4);
            obj.pee = zeros(3,6);
            obj.pee2b = zeros(3,6);
            obj.pin = zeros(3,6);
            obj.inv_jac = zeros(18,6);
            obj.leg = Leg.empty;
            for i = 1:6
                obj.leg(i) = Leg(leg_config, stroke, obj.leg_base_pe(i,:));
            end
        end
        
        function invKin(obj,peb,pee,sequence)
            %invKin 运动学反解，根据身体位姿和足尖坐标计算丝杠长度
            %   peb 身体位姿，1*6
            %   pee 足尖坐标，相对地面，3*6
            if nargin < 4
                sequence = 'ZXZ';
            end
            obj.peb = peb;
            obj.pmb = pe2pm(obj.peb,sequence);
            obj.pee = pee;
            pee2b_tmp = obj.pmb\[pee;ones(1,6)]; % 足尖坐标从地面坐标系变换到身体坐标系
            obj.pee2b = pee2b_tmp(1:3,:);
            for i = 1:6
                obj.leg(i).setPee(obj.pee2b(:,i),'B');
                obj.pin(:,i) = obj.leg(i).Pin;
                obj.q(:,i) = obj.leg(i).q;
            end
        end
        
        function calcInvJac(obj)
            %calcJac 计算身体雅可比的逆
            %   此处显示详细说明
            for i = 1:6
                obj.leg(i).calcVelocityJacobian();
                jacv = obj.leg(i).jac;
                R_b2h = obj.leg(i).base_pm(1:3,1:3);
                pf_cross = cross_v3(obj.leg(i).Pee_b);
                G_Bi = jacv\R_b2h'*[-eye(3) pf_cross];
                obj.inv_jac(3*i-2:3*i,:) = G_Bi;
            end
        end
        
        function setQ(obj, peb, q, init_pee)
            %forKin 运动学正解，根据身体位姿和丝杠长度计算足尖坐标
            %   此处显示详细说明
            obj.peb = peb;
            obj.pmb = pe2pm(obj.peb);
            for i = 1:6
                obj.leg(i).setQ(q(:,i), init_pee(:,i), 'B');
                obj.pee(:,i) = obj.leg(i).Pee_b;
            end
        end

        function bool_out = isInWorkspace(obj)
            %isInWorkspace 判断当前末端位置是否在工作空间内
            %   此处显示详细说明
            bool_out = true;
            for i = 1:6
                if ~obj.leg(i).isInWorkspace()
                    bool_out = false;
                    break;
                end
            end
        end

    end
end

