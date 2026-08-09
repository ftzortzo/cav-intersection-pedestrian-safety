% PLOT_CAV  Draw the current vehicle as a rotated rectangle.
%
% Renders vehicle i as a filled blue rectangle at its current pose. Done (exited)
% CAVs are drawn slightly faded. Runs in the caller's workspace; expects:
% rect_length, rect_width, CAVs, i.

vertices = [-rect_length/2, -rect_width/2;
             rect_length/2, -rect_width/2;
             rect_length/2,  rect_width/2;
            -rect_length/2,  rect_width/2]';

th = CAVs(i).theta;
R  = [cos(th), -sin(th); sin(th), cos(th)];
rv = R * vertices;
rv(1, :) = rv(1, :) + CAVs(i).x;
rv(2, :) = rv(2, :) + CAVs(i).y;

if CAVs(i).Done
    fill(rv(1, :), rv(2, :), [0.6 0.6 0.9], 'EdgeColor', 'none');
else
    fill(rv(1, :), rv(2, :), 'b');
end
