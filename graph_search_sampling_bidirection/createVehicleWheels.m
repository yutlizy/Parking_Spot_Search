function wheels = createVehicleWheels(vehicleDims, steer)

wheelLength = vehicleDims.Length/10;
wheelWidth  = vehicleDims.Length/50;
hw = vehicleDims.Width/2;
wb = vehicleDims.Wheelbase;

X = [-wheelLength/2 -wheelLength/2 wheelLength/2 wheelLength/2]';
Y = [-hw-wheelWidth -hw -hw -hw-wheelWidth]';

wheels = polyshape(X,Y);
wheels(end+1) = wheels(1).translate([0,vehicleDims.Width+wheelWidth]);
wheels(end+1) = wheels(1).translate([wb,0]);
wheels(end+1) = wheels(2).translate([wb,0]);
wheels(3) = rotateWheel(wheels(3), steer);
wheels(4) = rotateWheel(wheels(4), steer);
end