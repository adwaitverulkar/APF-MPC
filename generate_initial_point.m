function pnt = generate_initial_point()
    % Define the coordinates of the diagonal points for the three rectangles
    % Rectangle 1
    x1_1 = -1;  % x-coordinate of the first diagonal point for rectangle 1
    y1_1 = -4;  % y-coordinate of the first diagonal point for rectangle 1
    x2_1 = 1;  % x-coordinate of the second diagonal point for rectangle 1
    y2_1 = -2;  % y-coordinate of the second diagonal point for rectangle 1
    
    % Rectangle 2
    x1_2 = -3.5;  % x-coordinate of the first diagonal point for rectangle 2
    y1_2 = -1;  % y-coordinate of the first diagonal point for rectangle 2
    x2_2 = -1.5;  % x-coordinate of the second diagonal point for rectangle 2
    y2_2 = 1;  % y-coordinate of the second diagonal point for rectangle 2
    
    % Rectangle 3
    x1_3 = -1;  % x-coordinate of the first diagonal point for rectangle 3
    y1_3 = 2;  % y-coordinate of the first diagonal point for rectangle 3
    x2_3 = 1;  % x-coordinate of the second diagonal point for rectangle 3
    y2_3 = 4;  % y-coordinate of the second diagonal point for rectangle 3
    
    % Randomly choose one of the three rectangles
    random_rectangle = randi([1, 3]);
    
    % Depending on the chosen rectangle, calculate the minimum and maximum x and y values
    if random_rectangle == 1
        min_x = min(x1_1, x2_1);
        max_x = max(x1_1, x2_1);
        min_y = min(y1_1, y2_1);
        max_y = max(y1_1, y2_1);
    elseif random_rectangle == 2
        min_x = min(x1_2, x2_2);
        max_x = max(x1_2, x2_2);
        min_y = min(y1_2, y2_2);
        max_y = max(y1_2, y2_2);
    else
        min_x = min(x1_3, x2_3);
        max_x = max(x1_3, x2_3);
        min_y = min(y1_3, y2_3);
        max_y = max(y1_3, y2_3);
    end
    
    % Generate a random point inside the chosen rectangle
    random_x = min_x + (max_x - min_x) * rand();
    random_y = min_y + (max_y - min_y) * rand();

    pnt = [random_x, random_y];
    
    % Display the generated random point and the chosen rectangle
    fprintf('Random point inside Rectangle %d: (%.2f, %.2f)\n', random_rectangle, random_x, random_y);
    
    % % Optionally, you can plot the chosen rectangle and the random point
    % figure;
    % hold on;
    % 
    % % Plot the chosen rectangle
    % plot([x1_1, x2_1, x2_1, x1_1, x1_1], [y1_1, y1_1, y2_1, y2_1, y1_1], 'b-');
    % plot([x1_2, x2_2, x2_2, x1_2, x1_2], [y1_2, y1_2, y2_2, y2_2, y1_2], 'b-');
    % plot([x1_3, x2_3, x2_3, x1_3, x1_3], [y1_3, y1_3, y2_3, y2_3, y1_3], 'b-');
    % 
    % 
    % % Plot the random point
    % scatter(random_x, random_y, 'r', 'filled');
    % xlabel('X-axis');
    % ylabel('Y-axis');
    % xlim([-5 5])
    % ylim([-5 5])

end