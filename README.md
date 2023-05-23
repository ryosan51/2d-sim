# 2d-sim

function generate_route_candidates()

    % Example lane centerline points
    lane_centerline = [0, 0; 1, 1; 2, 2; 3, 3];

    % Generate route candidates
    candidates = generate_curved_candidates(lane_centerline);

    % Plot lane centerline
    plot(lane_centerline(:, 1), lane_centerline(:, 2), 'bo-');
    hold on;

    % Plot route candidates
    for i = 1:numel(candidates)
        candidate = candidates{i};
        plot(candidate(:, 1), candidate(:, 2), 'r--');
    end

    % Set plot properties
    xlabel('X');
    ylabel('Y');
    legend('Lane Centerline', 'Route Candidates');
    grid on;
    axis equal;
    hold off;

end

function candidates = generate_curved_candidates(centerline_points)
    candidates = {};

    % Define curvature radius
    curvature_radius = 1.0;

    % Generate curved route candidates
    for i = 1:size(centerline_points, 1)
        point = centerline_points(i, :);

        % Calculate the tangent vector at each centerline point
        tangent_vector = calculate_tangent_vector(centerline_points, i);

        % Generate the curved route candidate based on the tangent vector and curvature radius
        curved_candidate = generate_curved_candidate(point, tangent_vector, curvature_radius);
        candidates{end+1} = curved_candidate;
    end

end

function tangent_vector = calculate_tangent_vector(centerline_points, index)
    % Calculate the tangent vector at the current point using neighboring points
    % You can use numerical differentiation or spline interpolation, depending on the data and requirements
    % Return the tangent vector as a 2D vector or a unit vector

    % Example: Use forward differencing to estimate tangent vector
    if index < size(centerline_points, 1)
        next_point = centerline_points(index + 1, :);
        tangent_vector = next_point - centerline_points(index, :);
    else
        prev_point = centerline_points(index - 1, :);
        tangent_vector = centerline_points(index, :) - prev_point;
    end

    % Normalize the tangent vector
    tangent_vector = tangent_vector / norm(tangent_vector);

end

function curved_candidate = generate_curved_candidate(start_point, tangent_vector, curvature_radius)
    % Generate a curved route candidate given the start point, tangent vector, and curvature radius
    % Determine the number of points to generate based on the desired resolution and curve length
    % Use trigonometry and geometry to calculate the coordinates of the curve points
    % Return the curved route candidate as an Nx2 matrix of points

    % Define the number of points to generate for the curved route candidate
    num_points = 50; % Adjust this value based on the desired resolution

    % Calculate the control points for the curved route candidate
    control_points = [
        start_point;
        start_point + curvature_radius * tangent_vector;
        start_point + 2 * curvature_radius * tangent_vector;
        start_point + 3 * curvature_radius * tangent_vector
        % Add more control points as needed
    ];

    % Generate the spline curve
    t = linspace(0, 1, num_points);
    spline_points = spline(1:size(control_points, 1), control_points.', t).';

    curved_candidate = spline_points(:, 1:2);
end

% Call the main function
generate_route_candidates();
