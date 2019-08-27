classdef Hexapod < handle
    %Hexapod 六足机器人
    %   此处显示详细说明
    
    properties
        peb
        pmb
        pee
        pin
        leg_base_pe
        leg
    end
    
    methods
        function obj = Hexapod(leg_config,stroke)
            %hexapod 构造此类的实例
            %   此处显示详细说明
            obj.peb = zeros(1,6);
            obj.pmb = eye(4);
            obj.pee = zeros(3,6);
            obj.pin = zeros(3,6);
            obj.leg_base_pe = [ -0.43322 , 0 , -0.19907,   pi/2,  pi* 2/3,   -pi/2-pi*70/180;
                                -0.48305 , 0 ,  0,         pi/2,  pi* 3/3,   -pi/2-pi*70/180;
                                -0.43322 , 0 ,  0.19907,   pi/2,  pi* 4/3,   -pi/2-pi*70/180;
                                 0.43322 , 0 , -0.19907,   pi/2,  pi* 1/3,   -pi/2-pi*70/180;
                                 0.48305 , 0 ,  0,         pi/2,  pi* 0/3,   -pi/2-pi*70/180;
                                 0.43322 , 0 ,  0.19907,   pi/2,  pi* 5/3,   -pi/2-pi*70/180];
            obj.leg = Leg.empty;
            for i = 1:6
                obj.leg(i) = Leg(leg_config, stroke, obj.leg_base_pe(i,:));
            end

        end
        
        function invKin(obj,peb,pee)
            %invKin 运动学反解，根据身体位姿和足尖坐标计算丝杠长度
            %   此处显示详细说明
            obj.peb = peb;
            obj.pmb = pe2pm(obj.peb);
            obj.pee = pee;
            pee2b = obj.pmb\[pee;ones(1,6)]; % 足尖坐标从地面坐标系变换到身体坐标系
            pee2b = pee2b(1:3,:);
            for i = 1:6
                obj.leg(i).setPee(pee2b(:,i),'B');
                obj.pin(:,i) = obj.leg(i).Pin;
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

