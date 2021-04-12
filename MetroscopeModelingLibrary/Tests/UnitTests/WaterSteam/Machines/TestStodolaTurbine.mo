within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Machines;
model TestStodolaTurbine
  input Modelica.SIunits.AbsolutePressure P_source(start = 20e5);
  input Modelica.SIunits.SpecificEnthalpy h_source(start = 2.7718e6);
  input Modelica.SIunits.MassFlowRate Q(start = 100);

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine stodolaTurbine
    annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
equation

  // Forward causality
  source.P_out = P_source;
  source.h_vol = h_source;
  source.Q_out = -Q;
  sink.h_vol=1e6;
  stodolaTurbine.Cst=1458.5354;
  stodolaTurbine.area_nz=1;
  stodolaTurbine.eta_nz=1.0;
  stodolaTurbine.eta_is=0.82042;


  // Reverse causality
  // To determine eta_is, give the outlet enthalpy
  // To determine Cst, give the outlet pressure
  /*
  source.P_out = P_source;
  source.h_vol = h_source;
  source.Q_out = -Q;
  sink.h_vol=1e6;
  sink.P_in = 10e5;
  sink.h_in = 2.55e6;
  stodolaTurbine.area_nz=1;
  stodolaTurbine.eta_nz=1.0;
  */


  connect(stodolaTurbine.C_out, sink.C_in)
    annotation (Line(points={{6.2,0},{54,0}}, color={238,46,47}));
  connect(source.C_out, stodolaTurbine.C_in)
    annotation (Line(points={{-60,0},{-14,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},
            {80,20}})),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})));
end TestStodolaTurbine;
