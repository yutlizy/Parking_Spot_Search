function bodyShape = createEgoVehicleBody(vehicleDims)

% Create a polyshape object, with origin at rear-axle.
ro = vehicleDims.RearOverhang;
fo = vehicleDims.FrontOverhang;
wb = vehicleDims.Wheelbase;
hw = vehicleDims.Width/2;

% Create a polyshape object, with origin at rear-axle.
X = [-wb/2-ro wb/2+fo/2 wb/2+fo/2 -ro-wb/2]';
Y = [-hw   -hw    hw  hw]';
bodyShape = polyshape( X, Y);
end
