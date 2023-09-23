function g_APF = repulsive_pot_lane(x, y, RX, RY, d_0, nu)
    % Define the number of rectangles (n_rects) based on the size of RX or RY
    n_rects = size(RX, 1);

    % Initialize g_APF to zero
    g_APF = 0;

    for i = 1:n_rects
        rx1 = RX(i, 1);
        rx2 = RX(i, 2);
        ry1 = RY(i, 1);
        ry2 = RY(i, 2);

        % Calculate the distances and differences
        dx1 = rx1 - x;
        dx2 = x - rx2;
        dy1 = ry1 - y;
        dy2 = y - ry2;
    
        % Calculate the element-wise maximum of the three comparisons
        dx = max(max(dx1, zeros(size(x))), dx2);
        dy = max(max(dy1, zeros(size(y))), dy2);

        % disp(size(dx))

        % Calculate the APF term for the current rectangle
        % dx = max(rx1 - x, zeros(size(x)), x - rx2);
        % dy = max(ry1 - y, zeros(size(y)), y - ry2);
        g_APF = g_APF + nu * (exp(-((sqrt((dx^2) + (dy^2)) + d_0).^2) / (2 * d_0^2)));
        
        % disp(size(g_APF))
        % g_APF = g_APF + nu * sum(exp( -((sqrt((max([RX(i,1)- x, 0,  x-RX(i,2)])).^2 + ...
        %      (max([ RY(i,1)- y,  0,  y-RY(i,2)])).^2) + d_0)^2/(2*d_0^2))));
        % 

    end
end


