function [DHtable] = DH_generator(alpha,l,q)
DHtable = [pi/2  l(1)  0 q(1) ;
           -pi/2  l(2)  0 q(2)];
end