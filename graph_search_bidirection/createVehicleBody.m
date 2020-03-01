function bodyShape = createVehicleBody(vehicleDims)

% Create a polyshape object, with origin at rear-axle.
ro = vehicleDims.RearOverhang;
fo = vehicleDims.FrontOverhang;
wb = vehicleDims.Wheelbase;
hw = vehicleDims.Width/2;

% Create a polyshape object, with origin at rear-axle.
X = [-ro wb+fo wb+fo -ro]';
Y = [-hw   -hw    hw  hw]';
bodyShape = polyshape( X, Y);
end
