%% vehicle shape plot test
clear all;

vehicleDims.RearOverhang = 1;
vehicleDims.FrontOverhang = 1;
vehicleDims.Wheelbase = 2.5;
vehicleDims.Width = 2;
vehicleDims.Length = 4.5;
steer = 0;
vehiclePose = [1 1 90];

bodyShape   = createVehicleBody(vehicleDims);

axles       = createVehicleAxles(vehicleDims);
wheels      = createVehicleWheels(vehicleDims, steer);

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
    color = hAx.ColorOrder( hAx.ColorOrderIndex,: );
    hAx.ColorOrderIndex = hAx.ColorOrderIndex+1;
end

hShape = plot(hAx, bodyShape, ...
    'FaceColor', color, 'EdgeColor', color, 'AlignVertexCenters', 'on', ...
    'FaceAlpha', 0.5);

hold on;

hAxlesWheels = plot(hAx, [axles wheels] , 'FaceColor', 'k', 'FaceAlpha', 1);

% plot(bodyShape);
% hold on;
% plot(axles);
% hold on;
% plot(wheels);
% hold on;
axis equal;