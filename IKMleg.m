function out = IKMleg ( Sfdot,leg_label, IGM )
        % Computation of the inverse kinematic model of the robot (velocity)
        % INPUTS:
        % Sfdot: Velocity (vector of dimension 3) in BODY frame
        % Leg label: From 1 to 6 for geometric calculations
        % IGM: Structure output of the IGM at same time instant in which
        % IKM should be computed.
        
    R_U = sqrt(27^2+43^2)*sin(pi-asin(199/282)-atan2(43,27))+159; %六条腿U1所在的圆半径
    Sf = IGM.Sf;
    Sfdot = [Sfdot(1); Sfdot(2); Sfdot(3)];
    switch leg_label
        case 1 
            % Translation from B to L in B
            P_B_LORG = [R_U*sin(pi/6);0;R_U*cos(pi/6)];
            % Rotation of B wrt L (from B to L)
            R_L_B = [cos(-1/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(-1/3*pi)*sqrt(282^2-199^2)/282;cos(-1/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(-1/3*pi)*199/282;sin(-1/3*pi),0,cos(-1/3*pi)];
            % Foot position in L frame
            Sf_L = R_L_B*(Sf - P_B_LORG);
            % Rotate speed vector from B into L frame
            Sf_Ldot = R_L_B*Sfdot;
        case 2 
            P_B_LORG = [R_U;0;0];
            R_L_B = [cos(0)*sqrt(282^2-199^2)/282,-199/282,-sin(0)*sqrt(282^2-199^2)/282;cos(0)*199/282,sqrt(282^2-199^2)/282,-sin(0)*199/282;sin(0),0,cos(0)];
            Sf_L = R_L_B*(Sf - P_B_LORG);
            Sf_Ldot = R_L_B*Sfdot;
        case 3 
            P_B_LORG = [R_U*sin(pi/6);0;-R_U*cos(pi/6)];
            R_L_B = [cos(1/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(1/3*pi)*sqrt(282^2-199^2)/282;cos(1/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(1/3*pi)*199/282;sin(1/3*pi),0,cos(1/3*pi)];
            Sf_L = R_L_B*(Sf - P_B_LORG);
            Sf_Ldot = R_L_B*Sfdot;
        case 4 
            P_B_LORG = [-R_U*sin(pi/6);0;-R_U*cos(pi/6)];
            R_L_B = [cos(2/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(2/3*pi)*sqrt(282^2-199^2)/282;cos(2/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(2/3*pi)*199/282;sin(2/3*pi),0,cos(2/3*pi)];
            Sf_L = R_L_B*(Sf - P_B_LORG);
            Sf_Ldot = R_L_B*Sfdot;
        case 5 
            P_B_LORG = [-R_U;0;0];
            R_L_B = [cos(pi)*sqrt(282^2-199^2)/282,-199/282,-sin(pi)*sqrt(282^2-199^2)/282;cos(pi)*199/282,sqrt(282^2-199^2)/282,-sin(pi)*199/282;sin(pi),0,cos(pi)];
            Sf_L = R_L_B*(Sf - P_B_LORG);
            Sf_Ldot = R_L_B*Sfdot;
        otherwise %求腿6的Sf在腿坐标系(L)下的坐标
            P_B_LORG = [-R_U*sin(pi/6);0;R_U*cos(pi/6)];
            R_L_B = [cos(4/3*pi)*sqrt(282^2-199^2)/282,-199/282,-sin(4/3*pi)*sqrt(282^2-199^2)/282;cos(4/3*pi)*199/282,sqrt(282^2-199^2)/282,-sin(4/3*pi)*199/282;sin(4/3*pi),0,cos(4/3*pi)];
            Sf_L = R_L_B*(Sf - P_B_LORG);
            Sf_Ldot = R_L_B*Sfdot;
    end
    % Extract feet position in L frame
    x = Sf_L(1);
    y = Sf_L(2);
    z = Sf_L(3);
    % Express Cardan joint positions in L frame
    U2_L = [0;228;132];
    U3_L = [0;228;-132];  
    % Express ankle positions in A frame
    S2_A = [0;59;34];
            S2x = S2_A(1);
            S2y = S2_A(2);
            S2z = S2_A(3);
    S3_A = [0;59;-34];
            S3x = S3_A(1);
            S3y = S3_A(2);
            S3z = S3_A(3);
    % Get angles from the input IGM (actual configuration)
    a1 = IGM.alpha(1);
    b1 = IGM.beta(1);
    l1 = IGM.l(1);
    l2 = IGM.l(2);
    l3 = IGM.l(3);
    sa1 = sin(a1);
    ca1 = cos(a1);
    sb1 = sin(b1);
    cb1 = cos(b1);
    
    % Calculate Jacobian
    k21 = U2_L'*[-sa1*cb1, sa1*sb1, ca1; 0,0,0; -ca1*cb1, ca1*sb1, -sa1]*[S2x+l1; S2y; S2z];
    k22 = U2_L'*[-ca1*sb1, ca1*cb1, cb1; cb1,-sb1,0; sa1*sb1, sa1*cb1, 0]*[S2x+l1; S2y; S2z];
    k23 = U2_L'*[ca1*cb1;sb1;-sa1*cb1] - l1 - S2x;
    k31 = U3_L'*[-sa1*cb1, sa1*sb1, ca1; 0,0,0; -ca1*cb1, ca1*sb1, -sa1]*[S3x+l1; S3y; S3z];
    k32 = U3_L'*[-ca1*sb1, ca1*cb1, cb1; cb1,-sb1,0; sa1*sb1, sa1*cb1, 0]*[S3x+l1; S3y; S3z];
    k33 = U3_L'*[ca1*cb1;sb1;-sa1*cb1] - l1 - S3x;   
    J1 = [z, -y*ca1, ca1*cb1; 0, x*ca1-z*sa1, -x; -x, sa1, -sa1*cb1];
    J2 = [0,0,1;-k21/l2, -k22/l2, -k23/l2; -k31/l3, -k32/l3, -k33/l3 ];
    Jv = J1/(J2);  
    ldot = Jv\Sf_Ldot;
    
    % Compute alphadot and betadot
    temp = J2\ldot;
    alphadot = temp(1);
    betadot = temp(2);
    % Fill in output
    out.ldot = ldot;
    out.alphadot = alphadot;
    out.betadot = betadot;
    out.Sfdot = Sfdot;
    out.Sf_Ldot = Sf_Ldot;
end