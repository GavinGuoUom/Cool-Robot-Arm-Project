% Authors (so you know who to complain to if it breaks)
% Kinematics: Cavan and (Sam? I think?)
% Parabolic Trajectory: Han & Paddy
% Code making it all work together: Paddy

% Scale to work with arm
chess_scalar = 0.03;

% Positions
REST = convert_position([4.5 1.5 2], chess_scalar);
piece_init = convert_position([1 1 2], chess_scalar);
piece_fin = convert_position([2 4 2], chess_scalar);
x_lim_chess = [-0.12 0.12];
y_lim_chess = [0.105 0.345];

x_rand_start =  (x_lim_chess(2) - x_lim_chess(1))*unifrnd(0,1) + x_lim_chess(1);
x_rand_end = (x_lim_chess(2) - x_lim_chess(1))*unifrnd(0,1) + x_lim_chess(1);

y_rand_start = (y_lim_chess(2) - y_lim_chess(1))*unifrnd(0,1) + y_lim_chess(1);
y_rand_end = (y_lim_chess(2) - y_lim_chess(1))*unifrnd(0,1) + y_lim_chess(1);

disp(x_rand_start)
disp(x_rand_end)
disp(y_rand_start)
disp(y_rand_end)


piece_init = [x_rand_start y_rand_start 0.15];
piece_fin = [x_rand_end y_rand_end 0.15];

%=======================================
% List of Required Functions
%=======================================
% 1) move_piece DONE(ish)
% 2) take_piece
% 3) castle long/short
% 4) Pawn Promote
% 5) en_passant
%=======================================
% convert_position

%full_route = take_piece(piece_init, piece_fin, REST, chess_scalar);
full_route = move_piece(piece_init, piece_fin, REST, chess_scalar);

points = full_route;

% Number of points in full_route required to animation plays for correct
% length of time.
count = length(points);




% Kinematics Code
Arm_Model;

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = repmat(q0,count,1);

ik = inverseKinematics('RigidBodyTree', robot,'SolverAlgorithm','BFGSGradientProjection');
ik.SolverParameters.AllowRandomRestarts = false;
weights = [0, 0, 0, 1, 1, 1];
endEffector = 'body6';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

% Generate the chess board
ynumbers = chess_scalar*[3.5 4.5 5.5 6.5 7.5 8.5 9.5 10.5 11.5];
xnumbers = chess_scalar*[-4 -3 -2 -1 0 1 2 3 4];
board_squares = 0;

% Creates figure, robot and plots trajectory points.
figure
show(robot,qs(1,:));
view(2)
ax = gca;
xlim([-0.2, 0.2])
ylim([-0.1, 0.5])
zlim([-0.15, 0.3])
ax.Projection = 'orthographic';
hold on
plot3(points(:,1),points(:,2),points(:,3), color='k');
hold on 

[X,Y] = meshgrid(xnumbers, ynumbers);
Z = 0*X;
CData = [0   1   0   1        0    1    0    1    0
   1   0   1   0         1    0    1    0    1
   0   1   0   1         0    1    0    1    0
   1   0   1   0         1    0    1    0    1
   0   1   0   1         0    1    0    1    0
   1   0   1   0         1    0    1    0    1
   0   1   0   1         0    1    0    1    0
   1   0   1   0         1    0    1    0    1
   0   1   0   1         0    1    0    1    0];
colormap gray
s = mesh(X,Y,Z, CData, 'FaceAlpha', '0.5', 'EdgeColor', 'k');
s.FaceColor = 'flat';


framesPerSecond = 30;
r = rateControl(framesPerSecond);
i = 1;
for j = 0:(count)
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
    i = i + 1;
    if i == count
        i = 1; 
    end
end

%===========================================
%================FUNCTIONS==================
%===========================================

function[conv_pos] = convert_position(pos, chess_scalar)
    % Function to convert from chess coords to the coords the arm uses
    x = chess_scalar*(pos(1)-4.5);
    y = chess_scalar*(pos(2)+3);
    z = chess_scalar*(pos(3));
    conv_pos = horzcat(x,y,z);
end

function [step_vals] = step_size(val_1, val_2)
    % Function to find the step size, this is used in parabolic_path
	step_vals = ((val_2 - val_1)/50);
end


function [route_vals] = parabolic_path(start_pos, end_pos, chess_scalar)
    % Finds parabolic path between 2 positions

    % This is the primary pathfinding function, relies of function
    % stepsize to work. Able to find parabolic path between points of
    % equal z value.
	
	% Extract starting Positions
	xs = start_pos(1);
	ys = start_pos(2);
	zs = start_pos(3);

	% Extract final Positions
	xf = end_pos(1);
	yf = end_pos(2);
	zf = end_pos(3);

	% Values in each coordinate for the route
    % Generates an array of x coords using start and end point

    % if change in x = 0, generate array from 0's and add xs.
    if xs == xf
        N = 51;
        x_route = zeros(N,1) + xs;
    % Else xs != xf, find step size and generate array
    else
        x_route = (xs:(step_size(xs,xf)):xf)';
    end
        
    % Same for y
    if ys == yf
        N = 51;
        y_route = zeros(N,1) + ys;
    else
        y_route = (ys:(step_size(ys,yf)):yf)';
    end   

    % Parabolic Trajectory, honestly a mystery shout out to Han for this
    % one
    height_offset = calc_heights(start_pos, end_pos)
    arm_height = 0.04
    z_route = height_offset*(1 - (4/(xs^2 +xf^2 -2*xf*xs + yf^2 + ys^2 -2*yf*ys))*((x_route - (xs + xf)/2).^2 + (y_route - (ys + yf)/2).^2))';
    z_route = z_route + arm_height; % Raise points off chess board
    z_route = reshape(z_route, length(x_route), 1);
    
	% Combine into 1 matrix
	route_vals = horzcat(x_route, y_route, z_route);

end

function[full_drop_route] = drop_down(pos, drop)
    % THIS FUNCTION IS NOT CURRENTLY BEING USED, but may be useful in
    % future so have not deleted.
    
    % Generates an array of points to drop the arm down
    % to the board from the current position.
    x = pos(1);
    y = pos(2);
    z_start = pos(3);
    z_end = pos(3) - drop;
    
    z_route1 = (z_start:(step_size(z_start,z_end)):z_end)';
    z_route2 = (z_end:(step_size(z_end,z_start)):z_start)';
    
    N = size(z_route1);
    x_route = zeros(N(1),1) + x;
    y_route = zeros(N(1),1) + y;
    
    down_route_vals = horzcat(x_route,y_route,z_route1);
    up_route_vals = horzcat(x_route, y_route, z_route2);
    full_drop_route = vertcat(down_route_vals, up_route_vals);
end

function[full_route] = move_piece(int_pos, final_pos, REST, chess_scalar)
    % Code to produce a full array of the route to move a piece, produces 3
    % parabolic paths and combines them.
    route1 = parabolic_path(REST, int_pos, chess_scalar);
    route2 = parabolic_path(int_pos, final_pos, chess_scalar);
    route3 = parabolic_path(final_pos, REST, chess_scalar);
    
    full_route = vertcat(route1, route2, route3);
end

function[height] = calc_heights(init_pos, final_pos)
    % Code to produce calculate the max height of the parabola
    % based on the distance between start and end points
    dist_points = ((final_pos(1) - init_pos(1))^2 + (final_pos(2) - init_pos(2))^2 + (final_pos(3) - init_pos(3))^2)^(0.5);
    min_height = 0.03;
    height = min_height + dist_points/3.5 ;
end

function[full_route] = take_piece(int_pos, final_pos, REST, chess_scalar)
    % Code to produce a full array of the route required
    % by the arm to take another piece
    the_pit = chess_scalar*[5 6 4];
    route1 = move_piece(final_pos, the_pit, REST);
    route2 = move_piece(int_pos, final_pos, REST);
    full_route = vertcat(route1, route2);
end