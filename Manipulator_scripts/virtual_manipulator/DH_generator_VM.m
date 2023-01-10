function [DHtable] = DH_generator_VM(l_0,l, Q_aug)
    DHtable = [ pi/2 0 0 Q_aug(1); % --> first DoF of the spacecraft
                -pi/2 0 0 Q_aug(2); % --> second DoF of the spacecraft
                0 0 l_0 Q_aug(3); % --> third DoF of the spacecraft
                pi/2 0 l(1)  Q_aug(4) ;
                0  l(2) 0 Q_aug(5)];
   
end
