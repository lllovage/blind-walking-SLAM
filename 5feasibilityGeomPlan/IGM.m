function out  = IGM( Sf , leg_label )
%This function is the inverse kinematics solution of each leg of HEX Robot

%Sf is a 3-by-1 array, which represent the origin location of Foot frame in Body frame

% Radius of circle which pass origin locations of six Feet at initial state
% radius_Sf = 1503.26481494481730/2;
% %The initial origin location of Foot1 frame in Body frame
% P_B_Sf(:,1) = [sin(pi/6)*radius_Sf;-548.90171991133911;cos(pi/6)*radius_Sf];
% %The initial origin location of Foot2 frame in Body frame
% P_B_Sf(:,2) = [radius_Sf;-548.90171991133911;0];
% %The initial origin location of Foot3 frame in Body frame
% P_B_Sf(:,3) = [sin(pi/6)*radius_Sf;-548.90171991133911;-cos(pi/6)*radius_Sf];
% %The initial origin location of Foot4 frame in Body frame
% P_B_Sf(:,4) = [-sin(pi/6)*radius_Sf;-548.90171991133911;-cos(pi/6)*radius_Sf];
% %The initial origin location of Foot5 frame in Body frame
% P_B_Sf(:,5) = [-radius_Sf;-548.90171991133911;0];
% %The initial origin location of Foot6 frame in Body frame
% P_B_Sf(:,6) = [-sin(pi/6)*radius_Sf;-548.90171991133911;cos(pi/6)*radius_Sf];

%leg_label is the number of leg, which can be 1, 2, 3, 4, 5, 6

%l is a 3-by-1 array, l(i) represent the length of branch i of this leg, i can be 1, 2, 3.
%The allowed range of l(1) is [592.5, 912.5]
%The allowed range of l(2) and l(3) is [621.5, 941.5]

%alpha is a 3-by-1 array, alpha(i) represent the swinging angle of branch i of this leg, i can be 1, 2, 3
%The allowed range of alpha is [-pi/3, pi/3]

%beta is a 3-by-1 array, beta(i) represent the lifting angle of branch i of this leg, i can be 1, 2, 3
%The allowed range of beta is [-pi/4, pi/4]

%����������������˵��������˶�ѧ���������ϵ�����������������2.3��
%Sf��������˵���(S1)����������ϵ(B)�µ�����,leg_labelΪ�ȱ��
%lΪ˿�ܳ��ȣ�alphaΪ���ȷ����U��ת�ǣ�betaΪ̧�ȷ����U��ת��

%��SfдΪ��������ʽ
Sf = [Sf(1); Sf(2); Sf(3)];

%Feasbility flags proposed
feasFlags = [];
feasFlags.Fnl1 = 0;
feasFlags.Fnb1 = 0;
feasFlags.Fl1 = 0;
feasFlags.Fl2 = 0;
feasFlags.Fl3 = 0;
feasFlags.Fa1 = 0;
feasFlags.Fa2 = 0;
feasFlags.Fa3 = 0;
feasFlags.Fb1 = 0; 
feasFlags.Fb2 = 0;
feasFlags.Fb3 = 0;

%���������Sf��������ϵ(L)�µ�����
R_U = sqrt(27^2+43^2)*sin(pi-asin(199/282)-atan2(43,27))+159; %������U1���ڵ�Բ�뾶
switch leg_label
    case 1 %����1��Sf��������ϵ(L)�µ�����
        %L����ϵԭ����B����ϵ�µı�ʾ
        % Translation from B to L in B
        P_B_LORG = [R_U*sin(pi/6);0;R_U*cos(pi/6)];
        %B����ϵ��̬��L����ϵ�µı�ʾ
        % Rotation of B wrt L (from B to L)
        R_L_B = [cos(-1/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(-1/3*pi)*sqrt(282^2-199^2)/282;cos(-1/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(-1/3*pi)*199/282;sin(-1/3*pi),0,cos(-1/3*pi)];
        % Foot position in L frame
        Sf_L = R_L_B*(Sf - P_B_LORG);
    case 2 %����2��Sf��������ϵ(L)�µ�����
        P_B_LORG = [R_U;0;0];
        R_L_B = [cos(0)*sqrt(282^2-199^2)/282,-199/282,-sin(0)*sqrt(282^2-199^2)/282;cos(0)*199/282,sqrt(282^2-199^2)/282,-sin(0)*199/282;sin(0),0,cos(0)];
        Sf_L = R_L_B*(Sf - P_B_LORG);
    case 3 %����3��Sf��������ϵ(L)�µ�����
        P_B_LORG = [R_U*sin(pi/6);0;-R_U*cos(pi/6)];
        R_L_B = [cos(1/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(1/3*pi)*sqrt(282^2-199^2)/282;cos(1/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(1/3*pi)*199/282;sin(1/3*pi),0,cos(1/3*pi)];
        Sf_L = R_L_B*(Sf - P_B_LORG);
    case 4 %����4��Sf��������ϵ(L)�µ�����
        P_B_LORG = [-R_U*sin(pi/6);0;-R_U*cos(pi/6)];
        R_L_B = [cos(2/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(2/3*pi)*sqrt(282^2-199^2)/282;cos(2/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(2/3*pi)*199/282;sin(2/3*pi),0,cos(2/3*pi)];
        Sf_L = R_L_B*(Sf - P_B_LORG); %����Sf��B�µ��������Sf��L�µ�����
    case 5 %����5��Sf��������ϵ(L)�µ�����
        P_B_LORG = [-R_U;0;0];
        R_L_B = [cos(pi)*sqrt(282^2-199^2)/282,-199/282,-sin(pi)*sqrt(282^2-199^2)/282;cos(pi)*199/282,sqrt(282^2-199^2)/282,-sin(pi)*199/282;sin(pi),0,cos(pi)];
        Sf_L = R_L_B*(Sf - P_B_LORG);
    otherwise %����6��Sf��������ϵ(L)�µ�����
        P_B_LORG = [-R_U*sin(pi/6);0;R_U*cos(pi/6)];
        R_L_B = [cos(4/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(4/3*pi)*sqrt(282^2-199^2)/282;cos(4/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(4/3*pi)*199/282;sin(4/3*pi),0,cos(4/3*pi)];
        Sf_L = R_L_B*(Sf - P_B_LORG);
end
x = Sf_L(1);
y = Sf_L(2);
z = Sf_L(3);

%Sf�ڽ�������ϵ(����ϵA)�µ�����
Sf_A = [42,0,0];
Sfx = Sf_A(1);
Sfy = Sf_A(2);
Sfz = Sf_A(3);

%l1Ϊ��˿�ܳ��ȣ���ʽ�ο�����������(2-17)
% Assign feasibility flag Fnl1
    if (x^2 + y^2 + z^2 - Sfy^2 - Sfz^2) > 0
        l1 = sqrt(x^2 + y^2 + z^2 - Sfy^2 - Sfz^2) - Sfx;
        %U1��beta�ǣ���ʽ�ο�����������(2-21)
        % Assign feasibility flag Fnb1
        if -1 <= (y/sqrt((l1+Sfx)^2+Sfy^2)) && (y/sqrt((l1+Sfx)^2+Sfy^2)) <= 1 && ...
           -1 <= (Sfy/sqrt((l1+Sfx)^2+Sfy^2)) && (Sfy/sqrt((l1+Sfx)^2+Sfy^2)) <= 1
       
            b1 = asin(y/sqrt((l1+Sfx)^2+Sfy^2)) - asin(Sfy/sqrt((l1+Sfx)^2+Sfy^2));
            %U1��alpha�ǣ���ʽ�ο�����������(2-26)
            a1 = atan2(Sfz*x-((l1+Sfx)*cos(b1)-Sfy*cos(b1))*z,((l1+Sfx)*cos(b1)-Sfy*sin(b1))*x+Sfz*z);

            sa1 = sin(a1);
            ca1 = cos(a1);
            sb1 = sin(b1);
            cb1 = cos(b1);

            %S2�ڽ�������ϵ(����ϵA)�µ�����
            S2_A = [0;59;34];
            S2x = S2_A(1);
            S2y = S2_A(2);
            S2z = S2_A(3);

            %S3�ڽ�������ϵ(����ϵA)�µ�����
            S3_A = [0;59;-34];
            S3x = S3_A(1);
            S3y = S3_A(2);
            S3z = S3_A(3);

            %U2��������ϵ(����ϵL)�µ�����
            U2_L = [0;228;132];
            U2x = U2_L(1);
            U2y = U2_L(2);
            U2z = U2_L(3);

            %U3��������ϵ(����ϵL)�µ�����
            U3_L = [0;228;-132];
            U3x = U3_L(1);
            U3y = U3_L(2);
            U3z = U3_L(3);

            %S2��U2֮��ľ�����L����ϵx�᷽���ϵ�ͶӰ
            x2 = ca1*cb1*S2x - ca1*sb1*S2y + sa1*S2z + l1*ca1*cb1 - U2x;
            %S2��U2֮��ľ�����L����ϵy�᷽���ϵ�ͶӰ
            y2 = sb1*S2x + cb1*S2y + l1*sb1 - U2y;
            %S2��U2֮��ľ�����L����ϵz�᷽���ϵ�ͶӰ
            z2 = -sa1*cb1*S2x + sa1*cb1*S2y +ca1*S2z - l1*sa1*cb1 - U2z;
            %˿��2�ĳ���
            l2 = sqrt(x2^2 + y2^2 +z2^2);
            %U2��beta��
            b2 = asin(y2 / l2);
            %U2��alpha��
            a2 = -atan(z2 / x2);

            x3 = ca1*cb1*S3x - ca1*sb1*S3y + sa1*S3z + l1*ca1*cb1 - U3x;
            y3 = sb1*S3x + cb1*S3y + l1*sb1 - U3y;
            z3 = -sa1*cb1*S3x + sa1*cb1*S3y +ca1*S3z - l1*sa1*cb1 - U3z;
            l3 = sqrt(x3^2 + y3^2 +z3^2);
            b3 = asin(y3 / l3);
            a3 = -atan(z3 / x3);

            %˿�ܳ���
            % Assign feasibility flags Fl[1,2,3], Fa[1,2,3] and Fb[1,2,3]
            l = [l1;l2;l3];

            %alpha��
            alpha = [a1;a2;a3];
            %beta��
            beta = [b1;b2;b3];
            
            cond1 = l(1) >  912.5 | l(1) < 592.5;
            cond2 = l(2:3) >  941.5 | l(2:3) < 621.5;
            cond3 = alpha >  pi/3 | alpha < -pi/3;
            cond4 = beta >  pi/4 | beta < -pi/4;
            feasFlags.Fl1 = cond1;
            feasFlags.Fl2 = cond2(1);
            feasFlags.Fl3 = cond2(2);
            feasFlags.Fa1 = cond3(1);
            feasFlags.Fa2 = cond3(2);
            feasFlags.Fa3 = cond3(3);
            feasFlags.Fb1 = cond4(1);
            feasFlags.Fb2 = cond4(2);
            feasFlags.Fb3 = cond4(3);
            out.l= l; out.alpha = alpha; out.beta = beta; out.feasFlags = feasFlags;
            out.Sf = Sf; out.Sf_L = Sf_L;
        else
            feasFlags.Fnb1 = 1;
            l = [l1;nan(2,1)]; alpha = nan(1); beta = nan(1);
            out.l= l; out.alpha = alpha; out.beta = beta; out.feasFlags = feasFlags;
            out.Sf = Sf; out.Sf_L = Sf_L;
        end
    else
        feasFlags.Fnl1 = 1;
        l = nan(3,1); alpha = nan(3,1); beta = nan(3,1);
        out.l= l; out.alpha = alpha; out.beta = beta; out.feasFlags = feasFlags;
        out.Sf = Sf; out.Sf_L = Sf_L;
    end
end

