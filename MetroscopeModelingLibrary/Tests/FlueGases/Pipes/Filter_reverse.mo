within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model Filter_reverse
    extends Utilities.Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Utilities.Units;

    // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Parameters
  parameter Units.Height delta_z = 1;

  // Inputs for calibration
  input Real P_out(start=9) "barA";

  // Parameters for calibration
  output Units.FrictionCoefficient Kfr;
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{66,-10},{86,10}})));
  MetroscopeModelingLibrary.FlueGases.Pipes.Filter
                                                 filter
                                                      annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{26,-10},{46,10}})));
equation
    // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  // Parameters
  filter.delta_z = delta_z;

  // Inputs for calibration
  P_out_sensor.P_barA = P_out;

  // Parameters for calibration
  filter.Kfr = Kfr;
  connect(source.C_out, filter.C_in) annotation (Line(points={{-25,0},{-12,0}}, color={95,95,95}));
  connect(filter.C_out, P_out_sensor.C_in) annotation (Line(points={{8,0},{26,0}}, color={95,95,95}));
  connect(P_out_sensor.C_out,sink. C_in) annotation (Line(points={{46,0},{71,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Filter_reverse;
