within MetroscopeModelingLibrary.Power.Machines;
model Generator
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputYield eta(start=0.998) "Generator's efficiency";
  Units.OutletPower W_elec "Electrical power produced by the generator";
  Units.InletPower W_mech "Mechanical power received by the generator";

  Connectors.PowerInlet C_in annotation (Placement(transformation(extent={{-72,-10},{-52,10}}), iconTransformation(extent={{-72,-10},{-52,10}})));
  Connectors.PowerOutlet C_out annotation (Placement(transformation(extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,10}})));
equation 
  W_mech = C_in.W;
  W_elec + W_mech*eta = 0;
  C_out.W = W_elec;

  annotation (Diagram(coordinateSystem(extent={{-100,-60},{100,60}})),
                                Icon(coordinateSystem(extent={{-100,-60},{100,60}}),
                                     graphics={
        Rectangle(
          extent={{-56,33},{66,-33}},
          lineColor={0,0,0},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Line(points={{-38,-22},{-40,-26},{-42,-28},{-46,-30},{-50,-30},{-54,-28},{-58,-22},{-60,-14},{-60,-6},{-60,16},{-58,22},{-56,26},{-54,28},{-50,30},{-48,30},{-44,28},{-42,26},{-40,22},{-40,28},
              {-44,24},{-40,22}}),
        Polygon(
          points={{4,28},{-10,-2},{8,-6},{2,-28},{16,-2},{-2,2},{4,28}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Generator;
