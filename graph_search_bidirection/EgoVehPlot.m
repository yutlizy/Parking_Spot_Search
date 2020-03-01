function [hShape,hAxlesWheels] = EgoVehPlot(vehicleDims,vehiclePose,steer)

bodyShape   = createEgoVehicleBody(vehicleDims);

axles       = createEgoVehicleAxles(vehicleDims);
wheels      = createEgoVehicleWheels(vehicleDims, steer);

bodyShape   = moveToPose(bodyShape, vehiclePose);
axles       = moveToPose(axles, vehiclePose);
wheels      = moveToPose(wheels, vehiclePose);

carShapes = [bodyShape,axles,wheels];

bodyShape = carShapes(1);
axles     = carShapes(2:3);
wheels    = carShapes(4:end);

color = [];

hAx = newplot();
if isempty(color)
    % Get the next color in ColorOrder
    color = hAx.ColorOrder( 7,: );
%     hAx.ColorOrderIndex = randsample([1:7], 1);
    
end

if ishold(hAx)
    oldState = 'on';
else
    oldState = 'off';
end

% Turn on hold
hold(hAx, 'on')
restoreHoldState = onCleanup(@()hold(hAx, oldState));


hShape = plot(hAx, bodyShape, ...
    'FaceColor', color, 'EdgeColor', color, 'AlignVertexCenters', 'on', ...
    'FaceAlpha', 0.5);

hold on;

hAxlesWheels = plot(hAx, [axles wheels] , 'FaceColor', 'k', 'FaceAlpha', 1);


axis equal;

end