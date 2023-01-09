function [DHtable] = DH_generator_VM(l,r_0, Q_aug)
    
    DHtable = [ pi/2 0 0 Q_aug(1);
                -pi/2 0 0 Q_aug(2);
                0 0 r_0 Q_aug(3);
                pi/2 0 l(1)  Q_aug(4) ;
                0  l(2) 0 Q_aug(5)];
   
end