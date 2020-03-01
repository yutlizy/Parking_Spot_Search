function axles = createEgoVehicleAxles(vehicleDims)
axleLength = vehicleDims.Length/50;
hw = vehicleDims.Width/2;
wb = vehicleDims.Wheelbase;

% Create a polyshape object for rear and front axles.
X = [-axleLength/2-wb/2 -axleLength/2-wb/2 axleLength/2-wb/2 axleLength/2-wb/2]';
Y = [-hw hw hw -hw]';
axles  = polyshape(X, Y);
axles(end+1) = axles(1).translate([wb,0]);
end