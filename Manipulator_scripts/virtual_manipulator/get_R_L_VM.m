function [c2_j2000, R_, L_] = get_R_L_VM(T_lvlh_b,T_lvlh_j2000, d, p_j1_lvlh, p_EE_lvlh,p_tip_lvlh)
       % This function computes the R, L collections of vectors for the
       % virtual manipulator modelling.
       % INPUTs: DHTable is the Denavit-Hartenberg table, for the direct
       % kinematics, theta_v: spacecraft's attitude, T_lvlh_b: lvlh ---> base frame, d1/d2,l1: Robot dimensions.
       % OUTPUTs: center of mass c_2_j2000, R_, L_ expressed in the j2000
       % reference frame. trans is the homogeneous transformation of the
       % direct kinematic, it is requested for the optimization proble to
       % get the tip position.

         % Center of mass of the second joint (c_N, where N = 2)
      
        
       T_b_j2000 = T_lvlh_j2000*T_lvlh_b;  % from base to j2000

       % Get all positions of CoMs in J2000
       c0_j2000 = T_lvlh_j2000(1:3,4);
       
       c1_j2000 = T_b_j2000*[d(:,1);1];
       c1_j2000 = c1_j2000(1:3);

       c2_j2000 = T_b_j2000*[d(:,2);1];
       c2_j2000 = c2_j2000(1:3);
       
       d_tip = (p_EE_lvlh+p_tip_lvlh)/2;
       c_tip_j2000 = T_lvlh_j2000*[d_tip;1];
       c_tip_j2000 = c_tip_j2000(1:3);

       
       % Get all positions of joints in J2000
       p0_j2000 = T_b_j2000(1:3,4);
       
       p1_j2000 = T_lvlh_j2000*[p_j1_lvlh;1];
       p1_j2000 = p1_j2000(1:3);
       p_EE_j2000 = T_lvlh_j2000*[p_EE_lvlh;1];
       p_EE_j2000 = p_EE_j2000(1:3);
       p_tip_j2000 = T_lvlh_j2000*[p_tip_lvlh;1];
       p_tip_j2000 = p_tip_j2000(1:3);

       % Compute Ri forall i=0,1,2 in j2000
       R0_j2000 = p0_j2000 - c0_j2000;
       R1_j2000 = p1_j2000-c1_j2000;
       R2_j2000 = p_EE_j2000-c2_j2000;
       Rtip_j2000 = p_tip_j2000 - c_tip_j2000;

       % Compute Li forall i=1,2 in j2000
       L1_j2000 = c1_j2000-p0_j2000;
       L2_j2000 = c2_j2000-p1_j2000;
       Ltip_j2000 = c_tip_j2000 - p_EE_j2000;

% 
%        L_ = [L1_j2000'; L2_j2000'; Ltip_j2000';[0;0;0]'];
%        R_ = [R0_j2000';R1_j2000';R2_j2000'; Rtip_j2000'];

       L_ = [L1_j2000'; L2_j2000';[0;0;0]'];
       R_ = [R0_j2000';R1_j2000';R2_j2000'];
       

end
        