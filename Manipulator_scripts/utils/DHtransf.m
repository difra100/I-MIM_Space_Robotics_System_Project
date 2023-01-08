function T = DHtransf(line)

    T = [cos(line(4)) -cos(line(1))*sin(line(4)) sin(line(1))*sin(line(4)) line(2)*cos(line(4));
         sin(line(4)) cos(line(1))*cos(line(4)) -sin(line(1))*cos(line(4)) line(2)*sin(line(4));
         0 sin(line(1)) cos(line(1)) line(3);     
         0 0 0 1;];
end
