function divided_points = divide_points(pointa, pointb)
    % Calculate the length of each segment
    step = (pointb - pointa) / 8;

    % Initialise a 9x3 double variable
    divided_points = zeros(9, 3);

    % Record the coordinates of each point
    for i = 0:8
        divided_points(i + 1, :) = pointa + i * step;
    end
end
