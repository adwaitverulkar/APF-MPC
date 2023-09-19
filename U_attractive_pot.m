function g_goal = U_attractive_pot(x, y, X_goal, Y_goal, L, k)

    % Calculate the attractive APF for the goal location
    g_goal = L ./ (1 + exp(-k * sqrt((x - X_goal).^2 + (y - Y_goal).^2)));
end