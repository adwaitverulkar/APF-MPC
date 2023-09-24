function potential_field_fig = generate_APF_image(init_point, goal_pos, obs_loc, sigma_x, sigma_y, gaussian_amp, L_val, k_val, rx, ry, d_0_val, nu_val)

    a = linspace(-5,5,100); nx = length(a);
    b = linspace(-5,5,100); ny = length(b);
    
    [A,B] = meshgrid(a,b);
    
    potential_field = zeros(nx,ny);
    
    % Compute the values using vectorized operations
    potential_field = 1*U_attractive_pot(A, B, goal_pos(1), goal_pos(2),L_val,k_val) +...
                      1* repulsive_pot_lane(A, B, rx, ry, d_0_val, nu_val)- ...
                      1*invertedGaussian(A, B, gaussian_amp, goal_pos(1), goal_pos(2), sigma_x, sigma_y, 0.0) + ...
                      1*invertedGaussian(A, B, gaussian_amp, obs_loc(1), obs_loc(2), sigma_x, sigma_y, 0.0);


    figure = gcf;

    % Create the contour plot
    contourf(A, B, potential_field, 200, 'edgecolor', 'none');
    hold on

    % Define the center points where you want to add circles
    % centerPoints = [init_point; goal_pos.'];

    scatter(init_point(1), init_point(2),100, "blue", "filled", "o")
    scatter(goal_pos(1), goal_pos(2),100, "green", "filled", "o")
    
    % Define the radii of the circles
    % radii = [0.2, 0.2];

    % Define the color for the filled circles (e.g., red)
    % circleColor = 'r';
    % 
    % % Add filled circles at the center points
    % viscircles(centerPoints, radii, 'EdgeColor', circleColor, 'DrawBackgroundCircle', true);

    
    colormap(hot)
    % Show the plot
    axis equal;
    axis off;
    axis tight;
    % Optionally, save the plot as an image
    % saveas(gcf, 'contour_plot.png');


end