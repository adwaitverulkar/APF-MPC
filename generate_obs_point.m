function pnt = generate_obs_point()

    % Define the coordinates of the diagonal points
    x1 = -1;  % x-coordinate of the first diagonal point
    y1 = -1;  % y-coordinate of the first diagonal point
    x2 = 3;  % x-coordinate of the second diagonal point
    y2 = 1;  % y-coordinate of the second diagonal point
    
    % Calculate the minimum and maximum x and y values within the rectangle
    min_x = min(x1, x2);
    max_x = max(x1, x2);
    min_y = min(y1, y2);
    max_y = max(y1, y2);
    
    % Generate a random point inside the rectangle
    random_x = min_x + (max_x - min_x) * rand();
    random_y = min_y + (max_y - min_y) * rand();

    pnt = [random_x, random_y];
    
    % Display the generated random point
    fprintf('Random point inside the rectangle: (%.2f, %.2f)\n', random_x, random_y);
    
    % % Optionally, you can plot the rectangle and the random point
    % figure;
    % hold on;
    % plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1], 'b-');  % Plot the rectangle
    % scatter(random_x, random_y, 'r', 'filled');  % Plot the random point
    % xlabel('X-axis');
    % ylabel('Y-axis');
    % xlim([-5 5])
    % ylim([-5 5])


end