function minimum_covering_circle(robot_quantity, half_range)
% This is the test program of minimum covering circle algorithms
% method used here:
% first generate convex hull using gift wrapping algorithm
% then find the minimum circle by the convex hull and plot it

% robot_quantity is the number of dots
% half_range is the distribution range of the dots

% gift wrapping algorithm
% https://en.wikipedia.org/wiki/Gift_wrapping_algorithm
% algorithm of calculating circle from three points
% http://stackoverflow.com/questions/4103405/what-is-the-algorithm-for-finding-the-center-of-a-circle-from-three-points
% algorithm find minimum covering circle from convex hull
% http://www.personal.kent.edu/~rmuhamma/Compgeometry/MyCG/CG-Applets/Center/centercli.htm

% although it can be modified so that it is compatible with robot quantity of 2 or 1
% or multiple robots at the same line, it has no such mechanism temperarily

% demo test:
% minimum_covering_circle(20,0.5)
% minimum_covering_circle(100,1.0)

% randomly generate the dataset
pd = makedist('uniform','lower',-half_range,'upper',half_range);
position=random(pd,robot_quantity,2);  % 2d positions

% find the leftmost point as the starting point of the convex
convex_index(1) = 1;
for i=2:robot_quantity
    if position(i,1) < position(convex_index(1),1)
        convex_index(1) = i;
    end
end

% find second point on the convex hull
max_angle = 0;
for i=1:length(position)
    if i == convex_index(1)
        % jump over this point, it's itself
        continue;
    end
    base_vector = [0,-1];
    probe_vector = position(i,:) - position(convex_index(1),:);
    probe_angle = acos(dot(base_vector,probe_vector)/norm(probe_vector));
    if probe_angle > max_angle
        max_angle = probe_angle;
        convex_index(2) = i;  % the second point
    end
end

% continue find the rest convex points
% the only difference with finding the first one is the base_vector
convex_index_index = 2;  % current index of convex_index
convex_index_next = convex_index(2);  % so that it's not equal to first index
while (convex_index_next ~= convex_index(1))
    % next convex index is not equal to the first convex index
    max_angle = 0;
    base_vector = position(convex_index(convex_index_index-1),:) -...
        position(convex_index(convex_index_index),:);
    for i=1:length(position)
        if i == convex_index(convex_index_index)
            % exclude itself
            continue;
        end
        probe_vector = position(i,:) - position(convex_index(convex_index_index),:);
        probe_angle = acos(dot(base_vector,probe_vector)/norm(base_vector)/norm(probe_vector));
        if probe_angle > max_angle
            max_angle = probe_angle;
            convex_index(convex_index_index+1) = i;  % update the next convex index
        end
    end
    convex_index_next = convex_index(convex_index_index+1);  % the found index
    convex_index_index = convex_index_index+1;  % increment index of convex_index by 1
end
% if here then the convex is closed
% the last index is equal to the first index of convex_index
convex_index(end) = [];  % delete the repeated index

% find minimum covering circle based on the convex hull
% this algorithm is the purpose of this test program
circle_mode = 0;  % 2 means 2 points connecting the diameter, 3 means 3 points on the circle
two_points_index = zeros(1,2);
three_points_index = zeros(1,3);
% start with a random side of the convex
side_index = [1,2];  % the first side of the convex, they are index of convex_index
while true
    % disp('goes into the iteration');  % check how many iterations needed
    % find the smallest angle subtended the side
    smallest_angle = pi;
    convex_index_index = 3;  % the vertex subtend the side
    for i=1:length(convex_index)
        if i == side_index(1) || i == side_index(2)
            % exclude the vertices forming the side
            continue;
        end
        vector_1 = position(convex_index(side_index(1)),:) - position(convex_index(i),:);
        vector_2 = position(convex_index(side_index(2)),:) - position(convex_index(i),:);
        subtended_angle = acos(dot(vector_1,vector_2)/norm(vector_1)/norm(vector_2));
        if subtended_angle < smallest_angle
            smallest_angle = subtended_angle;
            convex_index_index = i;
        end
    end
    if smallest_angle >= pi/2
        % return the side as the diameter of the circle
        circle_mode = 2;
        two_points_index = convex_index(side_index);
        break;
    else
        % check the other two angle of the triangle from the side and the vertex
        % continue new iteration if one angle is obtuse
        % check the angle at vertex side_index(1)
        vector_1 = position(convex_index(convex_index_index),:) -...
            position(convex_index(side_index(1)),:);
        vector_2 = position(convex_index(side_index(2)),:) -...
            position(convex_index(side_index(1)),:);
        subtended_angle = acos(dot(vector_1,vector_2)/norm(vector_1)/norm(vector_2));
        if subtended_angle > pi/2
            side_index = [side_index(2),convex_index_index];
            continue;
        end
        % check the angle at vertex side_index(2)
        vector_1 = position(convex_index(convex_index_index),:) -...
            position(convex_index(side_index(2)),:);
        vector_2 = position(convex_index(side_index(1)),:) -...
            position(convex_index(side_index(2)),:);
        subtended_angle = acos(dot(vector_1,vector_2)/norm(vector_1)/norm(vector_2));
        if subtended_angle > pi/2
            side_index = [side_index(1),convex_index_index];
            continue;
        end
        % if here no vertices are obtuse
        % return the three points as the triangle of the circle
        circle_mode = 3;
        three_points_index = [convex_index(side_index),convex_index(convex_index_index)];
        break;
    end
end

% find center and radius of the circle
circle_center = zeros(1,2);
circle_radius = 0;
if circle_mode == 2
    % center of the two points
    ii = two_points_index;  % for abbreviation
    circle_center = (position(ii(1),:) + position(ii(2),:))/2;
    circle_radius = norm(position(ii(1),:) - position(ii(2),:))/2;
elseif circle_mode == 3
    % circle passing the three points
    iii = three_points_index;  % for abbreviation
    y_delta_a = position(iii(2),2) - position(iii(1),2);
    x_delta_a = position(iii(2),1) - position(iii(1),1);
    y_delta_b = position(iii(3),2) - position(iii(2),2);
    x_delta_b = position(iii(3),1) - position(iii(2),1);
    a_slope = y_delta_a/x_delta_a;
    b_slope = y_delta_b/x_delta_b;
    circle_center(1) = (a_slope*b_slope*(position(iii(1),2)-position(iii(3),2))+...
        b_slope*(position(iii(1),1)+position(iii(2),1))-...
        a_slope*(position(iii(2),1)+position(iii(3),1))) / (2*(b_slope-a_slope));
    circle_center(2) = -1*(circle_center(1)-(position(iii(1),1)+...
        position(iii(2),1))/2)/a_slope + (position(iii(1),2)+position(iii(2),2))/2;
    circle_radius = norm(circle_center - position(iii(1),:));
end


% plot the result
figure;
plot(position(:,1),position(:,2),'*k'); hold on;  % all the points
range = half_range*1.5;
axis([-range,range,-range,range]);
% the convex hull
for i=1:length(convex_index)-1
    plot(position([convex_index(i),convex_index(i+1)],1),...
        position([convex_index(i),convex_index(i+1)],2),'-k');
end
plot(position([convex_index(1),convex_index(end)],1),...
    position([convex_index(1),convex_index(end)],2),'-k');
% plot the important vertices
if circle_mode == 2
    plot(position(two_points_index,1),position(two_points_index,2),'-r');
elseif circle_mode ==3
    plot(position(three_points_index([1,2]),1), position(three_points_index([1,2]),2),'-r');
    plot(position(three_points_index([1,3]),1), position(three_points_index([1,3]),2),'-r');
    plot(position(three_points_index([2,3]),1), position(three_points_index([2,3]),2),'-r');
end
% plot the circle
rectangle('Position',[circle_center(1)-circle_radius,circle_center(2)-circle_radius,...
    circle_radius*2,circle_radius*2],'Curvature',[1,1]);
axis equal;

% display computation result of circle center and radius
circle_center
circle_radius


end

