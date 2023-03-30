% Define the coordinates of the four points
pointc = [1.0, 0.5, 0.5];
pointd = [0.5, 0.5, 0.1];
pointa = [0.5, -0.5, 0.1];
pointb = [1, -0.5, 0.5];

% Create a new graphics window
figure(2);

% Use the fill3 function to draw and colour a closed plane consisting of four points
fill3([pointa(1), pointb(1), pointc(1), pointd(1)], ...
      [pointa(2), pointb(2), pointc(2), pointd(2)], ...
      [pointa(3), pointb(3), pointc(3), pointd(3)],"g");

% Set axis labels and titles
xlabel('x');
ylabel('y');
zlabel('z');
title('Robot workspace');
