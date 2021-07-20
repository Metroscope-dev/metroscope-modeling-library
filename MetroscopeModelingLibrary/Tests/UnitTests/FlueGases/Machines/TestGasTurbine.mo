within MetroscopeModelingLibrary.Tests.UnitTests.FlueGases.Machines;
model TestGasTurbine
  import MetroscopeModelingLibrary;
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink(h_in(start = 283945),h_vol(start = 283945))
    annotation (Placement(transformation(extent={{66,70},{86,90}})));
  MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor              airCompressor(h_out(start = 695988))
    annotation (Placement(transformation(extent={{-34,70},{-14,90}})));
  MetroscopeModelingLibrary.FlueGases.Machines.FlueGasesTurbine flueGasesTurbine
    annotation (Placement(transformation(extent={{26,70},{46,90}})));
  MetroscopeModelingLibrary.FlueGases.Machines.CombustionChamber
    combustionChamber
    annotation (Placement(transformation(extent={{-6,70},{14,90}})));
  MetroscopeModelingLibrary.Fuel.PressureLosses.SingularPressureLoss
    fuelSingularPressureLoss(C_in(h_vol(start=935151)), rhom(start=13.7479,
        displayUnit="g/cm3"))
    annotation (Placement(transformation(extent={{-30,44},{-10,64}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source sourceFuel(h_vol(start=935151))
    annotation (Placement(transformation(extent={{-74,44},{-54,64}})));
  MetroscopeModelingLibrary.MoistAir.Converters.MoistAirToFlueGases
    moistAirToFlueGases
    annotation (Placement(transformation(extent={{-66,70},{-46,90}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-100,70},{-80,90}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Sink sink1
    annotation (Placement(transformation(extent={{56,90},{64,98}})));
equation

  // Forward causality

  source.P_out = 0.989e5;
  source.Q_out = -654.95;
  source.T_vol = 8 + 273.15;
  source.relative_humidity = 0.9;
  sourceFuel.P_out = 30e5;
  sourceFuel.Q_out = -15;
  sourceFuel.T_vol = 170+273.15;
  sourceFuel.Xi_vol = {0.90,0.05,0,0,0.025,0.025};


  combustionChamber.LHV =48130*1e3;
  fuelSingularPressureLoss.Kfr = 1e-6;
  airCompressor.tau = 17;
  airCompressor.eta_is = 0.9;
  combustionChamber.P_in-combustionChamber.P_out=0.05*combustionChamber.P_in;
  flueGasesTurbine.tau=16;
  flueGasesTurbine.eta_is=0.9;
  flueGasesTurbine.eta_mech = 1;


  sink.h_vol = 1e6;
  sink.Xi_vol = {0.768,0.232,0.0,0.0,0.0};




  // Reverse causality
  // The constants and yields of the compressor and turbine are determined
  /*
  source.P_out = 0.989e5;
  source.Q_out = -654.95;
  source.T_vol = 8 + 273.15;
  source.relative_humidity = 0.9;
  sourceFuel.P_out = 30e5;
  sourceFuel.Q_out = -15;
  sourceFuel.T_vol = 170+273.15;
  sourceFuel.Xi_vol = {0.90,0.05,0,0,0.025,0.025};
  combustionChamber.LHV =48130*1e3;
  fuelSingularPressureLoss.Kfr = 1e-6;
  airCompressor.P_out = 16.98e5;
  airCompressor.T_out = 406.6 +273.15;
  combustionChamber.P_in-combustionChamber.P_out=0.05*combustionChamber.P_in;
  sink.P_in = 1e5;
  sink.T_in = 600 +273.15;
  sink.h_vol = 1e6;
  sink.Xi_vol = {0.768,0.232,0.0,0.0,0.0};
  flueGasesTurbine.eta_mech = 1;
  */

  connect(flueGasesTurbine.C_out, sink.C_in)
    annotation (Line(points={{46.2,80},{66,80}}, color={238,46,47}));
  connect(airCompressor.C_out, combustionChamber.C_in)
    annotation (Line(points={{-13.8,80},{-6,80}}, color={238,46,47}));
  connect(combustionChamber.C_out, flueGasesTurbine.C_in)
    annotation (Line(points={{14.2,80},{26,80}}, color={238,46,47}));
  connect(airCompressor.Wmech, flueGasesTurbine.Wmech_compressor) annotation (
      Line(points={{-12.6,88.6},{-12,88.6},{-12,96},{20,96},{20,88.6},{24.6,
          88.6}},
        color={0,0,127}));
  connect(sourceFuel.C_out, fuelSingularPressureLoss.C_in)
    annotation (Line(points={{-54,54},{-30,54}}, color={238,46,47}));
  connect(fuelSingularPressureLoss.C_out, combustionChamber.C_fuel)
    annotation (Line(points={{-9.8,54},{4,54},{4,70}}, color={238,46,47}));
  connect(moistAirToFlueGases.C_out, airCompressor.C_in)
    annotation (Line(points={{-46,80},{-34,80}}, color={63,81,181}));
  connect(source.C_out, moistAirToFlueGases.C_in)
    annotation (Line(points={{-80,80},{-65.8,80}}, color={63,81,181}));
  connect(flueGasesTurbine.C_power, sink1.u) annotation (Line(points={{47.4,
          88.6},{46,88.6},{46,94},{56.4,94}}, color={0,0,127}));
    annotation (Placement(transformation(extent={{-16,42},{-4,54}})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,40},
            {100,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,40},{100,100}})));
end TestGasTurbine;
