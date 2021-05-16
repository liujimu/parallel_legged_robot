classdef Hexapod < handle
    %Hexapod ���������
    %   �˴���ʾ��ϸ˵��
    
    properties
        peb
        pmb
        pee
        pee2b
        pin
        q
        leg_base_pe
        leg
        body_jac_inv %�����ſɱȵ���
    end
    
    methods
        function obj = Hexapod(leg_config,stroke,leg_base_pe)
            %hexapod ��������ʵ��
            %   �˴���ʾ��ϸ˵��
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
            obj.body_jac_inv = zeros(18,6);
            obj.leg = Leg.empty;
            for i = 1:6
                obj.leg(i) = Leg(leg_config, stroke, obj.leg_base_pe(i,:));
            end
        end
        
        function invKin(obj,peb,pee,sequence)
            %invKin �˶�ѧ���⣬��������λ�˺�����������˿�ܳ���
            %   peb ����λ�ˣ�1*6
            %   pee ������꣬��Ե��棬3*6
            if nargin < 4
                sequence = 'ZXZ';
            end
            obj.peb = peb;
            obj.pmb = pe2pm(obj.peb,sequence);
            obj.pee = pee;
            pee2b_tmp = obj.pmb\[pee;ones(1,6)]; % �������ӵ�������ϵ�任����������ϵ
            obj.pee2b = pee2b_tmp(1:3,:);
            for i = 1:6
                obj.leg(i).setPee(obj.pee2b(:,i),'B');
                obj.pin(:,i) = obj.leg(i).Pin;
                obj.q(:,i) = obj.leg(i).q;
            end
        end
        
        function calcInvJac(obj)
            %calcJac ���������ſɱȵ���
            %   �˴���ʾ��ϸ˵��
            for i = 1:6
                obj.leg(i).calcVelocityJacobian();
                jacv = obj.leg(i).jac;
                R_b2h = obj.leg(i).base_pm(1:3,1:3);
                pf_cross = cross_v3(obj.leg(i).Pee_b);
                G_Bi = jacv\R_b2h'*[-eye(3) pf_cross]; %
                obj.body_jac_inv(3*i-2:3*i,:) = G_Bi;
            end
        end
        
        function output = calcPayload(obj)
            %calcPayload ������������ά�ȵĸ�������
            Fa = 3313; %˿���������
            payload = zeros(6,1);
            J_B = pinv(obj.body_jac_inv);
            for i = 1:6
                F_body = zeros(6,1);
                F_body(i) = 1;
                F_in = J_B'*F_body;%����ؽ���
                payload(i) = Fa/max(F_in);
            end
            output = payload;
        end
        
        function output = calcPayload3Leg(obj)
            %calcPayload3Leg ��������վ��ʱ��������ά�ȵĸ�������
            Fa = 3313; %˿���������
            payload = zeros(6,1);
            G_B1 = obj.body_jac_inv([1:3,7:9,13:15],:);
            J_B = pinv(G_B1);
            for i = 1:6
                F_body = zeros(6,1);
                F_body(i) = 1;
                F_in = J_B'*F_body;%����ؽ���
                payload(i) = Fa/max(F_in);
            end
            output = payload;
        end
        
        function setQ(obj, peb, q, init_pee)
            %forKin �˶�ѧ���⣬��������λ�˺�˿�ܳ��ȼ����������
            %   �˴���ʾ��ϸ˵��
            obj.peb = peb;
            obj.pmb = pe2pm(obj.peb);
            for i = 1:6
                obj.leg(i).setQ(q(:,i), init_pee(:,i), 'B');
                obj.pee(:,i) = obj.leg(i).Pee_b;
            end
        end

        function bool_out = isInWorkspace(obj)
            %isInWorkspace �жϵ�ǰĩ��λ���Ƿ��ڹ����ռ���
            %   �˴���ʾ��ϸ˵��
            bool_out = true;
            for i = 1:6
                if ~obj.leg(i).isInWorkspace()
                    bool_out = false;
                    break;
                end
            end
        end

        function plotRobot(obj)
            %plotRobot ���ƻ�����ʾ��ͼ
            %   �˴���ʾ��ϸ˵��
            leg_base_ori = obj.leg_base_pe(:,1:3)'; %���Źؽ�����
            leg_base_ori(4,:) = ones(1,6);
            hip_centers = obj.pmb*leg_base_ori;
            hip_centers(4,:) = [];
            body_hexagon = [hip_centers(:,1:3), hip_centers(:,6:-1:4)];
            hold on
            patch(body_hexagon(1,:),body_hexagon(3,:),body_hexagon(2,:),'r','LineWidth',1.5) %������������
            for i = 1:6
                leg_line = [hip_centers(:,i), obj.pee(:,i)];
                plot3(leg_line(1,:),leg_line(3,:),leg_line(2,:),'k','LineWidth',1.5) %��������
            end
            axis equal
        end
        
    end
end

