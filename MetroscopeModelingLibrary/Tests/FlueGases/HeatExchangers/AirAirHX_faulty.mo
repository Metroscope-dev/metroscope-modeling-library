within MetroscopeModelingLibrary.Tests.FlueGases.HeatExchangers;
model AirAirHX_faulty "MML4 isolated LiqLiqHX faulty test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FlueGasesTestIcon;
  extends MetroscopeModelingLibrary.Tests.FlueGases.HeatExchangers.AirAirHX_direct(AirAirHX(faulty=true));

  // MML4 interfaces declare quantities; isolated component at illustrative starts.
  input MetroscopeModelingLibrary.Utilities.Units.Percentage fault_fouling(start=20);

equation

  AirAirHX.fouling = fault_fouling;

  annotation (
    experiment(StopTime=1, Tolerance=1e-6),

    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-300,-180},{300,240}})),
    Documentation(info="<html><p>MML4 isolated component suite, 2026-10-20. Target local fix-bug-reheater HEAD 126f351e652e73df0eab808bbb7b29f9cb5e556f plus existing workspace changes, MSL 4.0.0. Approved graphical patterns: AirCompressor, MetroscopiaCCGT and MetroscopiaNPP. Native or approved icon dimensions retained.</p><p>Operating points and characteristic values are illustrative engineering sizing, not calibrated plant data. External inputs use existing sensor starts; ordinary enthalpy/fault inputs retain typed declarations and equations. Faults are constant. One Check Model and at most one simulation attempt; see the accompanying report for actual outcomes. No failed-model correction or rerun.</p></html>"));
end AirAirHX_faulty;
