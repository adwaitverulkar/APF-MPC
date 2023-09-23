function result = invertedGaussian(x, y, A, x0, y0, sigma_x, sigma_y, C)
    if nargin < 8
        C = 0;  % Default constant offset is zero
    end

    result = A * exp(-((x - x0).^2 / (2 * sigma_x^2)) - ((y - y0).^2 / (2 * sigma_y^2))) + C;
end