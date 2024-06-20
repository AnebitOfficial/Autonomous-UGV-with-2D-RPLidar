close all; clear all;
x = [];
y = [];
z = [];

x2D = [];
y2D = [];

map_data = 'map_data.txt';
map_info = 'map_info.txt';
data = readtable(map_data);
data = table2array(data(:,3:5));
info = readtable(map_info);
info = table2array(info);

number_of_points = height(data);
number_of_locations = height(info);

dphi = 0;
k = 1;
for i = 1:number_of_locations
    dx = info(i,1)*1000;
    dy = info(i,2)*1000;
    dteta = info(i,3);
    n = info(i,4);
    for j = k:n+k
        r = sqrt(data(j,2)^2 + 20^2);
        teta = data(j,1);
        phi = data(j,3);
        a = 2000;
        if r>500
            phi = phi - 90;
            z_ = r * sind(phi) * sind(teta);  
            x_ = -r * cosd(teta)  ;
            y_ = r * sind(teta)  * cosd(phi) ;
            %x_ = x_ +dx;
            %y_ = y_ +dy;
            yi = x_ * sind(dteta) + y_ * cosd(dteta)+dy;
            xi = x_ * cosd(dteta) - y_ * sind(dteta)+dx;

            zf = z_ * cosd(dphi) - xi * sind(dphi);
            xf = xi * cosd(dphi) + z_ * sind(dphi);

            if (z_ > -250) && (z_ < 2000) 
                x(end+1) = xf;
                y(end+1) = yi;
                z(end+1) = zf;
            end

            if phi == 0
                x2D(end+1) = xf;
                y2D(end+1) = yi;
            end

        end
    end
    k = n+k+1;
end


figure;
scatter3(x,y, z, 50,z, 'filled');  % 10 is the marker size
colormap('jet');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Map');
grid on;
axis equal; 
hold on;
view(3);

%xlim([-4000,4000]);
%ylim([-4000,4000]);
%zlim([-6000,6000]);

for i = 1:number_of_locations
    dx = info(i,1)*1000;
    dy = info(i,2)*1000;
    dteta = info(i,3);
    z_ = 0;
    dx_new = cosd(dteta)*1000;
    dy_new = sind(dteta)*1000;
    quiver3(dx, dy, z_, dx_new, dy_new, z_,1, 'LineWidth', 5, 'MaxHeadSize', 2);
end

dcm_obj = datacursormode(gcf);
set(dcm_obj, 'SnapToDataVertex','on','Enable','on');
dcm_obj.DisplayStyle = 'datatip';

% Initialize a global variable to store selected point coordinates
global pointStorage handleStorage;
pointStorage = [];
handleStorage = struct('line', [], 'text', []);

% Set a custom update function for datatips
set(dcm_obj, 'UpdateFcn', @customDataTipUpdate);

x_new = x((-100 < z) & (z < 400));
y_new = y((-100 < z) & (z < 400));

figure;
scatter(x2D,y2D,10,'filled');
title('2D Projection on Lidar Level');
xlabel('X-axis');
ylabel('Y-axis');
grid on;
hold on;

for i = 1:number_of_locations
    dx = info(i,1)*1000;
    dy = info(i,2)*1000;
    dteta = info(i,3);
    dx_new = cosd(dteta)*1000;
    dy_new = sind(dteta)*1000;
    quiver(dx, dy, dx_new, dy_new,1, 'LineWidth', 5, 'MaxHeadSize', 2);
end


function txt = customDataTipUpdate(~, event_obj)
    % Access global variables
    global pointStorage handleStorage;
    
    % Get the position of the selected datatip
    pos = get(event_obj, 'Position');
    
    % Store the position
    pointStorage = [pointStorage; pos];
    
    % Check if two points have been selected
    if size(pointStorage, 1) == 2
        % Calculate the Euclidean distance
        distance = sqrt(sum((pointStorage(1,:) - pointStorage(2,:)).^2));
        
        % Delete previous line and text if they exist
        if ~isempty(handleStorage.line)
            delete(handleStorage.line);
            delete(handleStorage.text);
        end
        
        % Plot a line between the two points
        hold on;
        handleStorage.line = plot3([pointStorage(1,1), pointStorage(2,1)], [pointStorage(1,2), pointStorage(2,2)], [pointStorage(1,3), pointStorage(2,3)], 'k--', 'LineWidth', 2);
        % Display the distance
        midPoint = mean(pointStorage, 1);
        handleStorage.text = text(midPoint(1), midPoint(2), midPoint(3), sprintf('Distance: %.2f cm', distance/10), 'FontSize', 15, 'Color', 'red');
        hold off;
        
        % Clear the stored points for next measurement
        pointStorage = [];
    end
    
    % Provide output text for the datatip
    txt = {['X: ', num2str(pos(1))], ['Y: ', num2str(pos(2))], ['Z: ', num2str(pos(3))]};
end