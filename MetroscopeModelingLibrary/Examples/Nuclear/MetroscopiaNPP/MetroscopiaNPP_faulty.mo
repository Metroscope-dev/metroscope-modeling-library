within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_faulty
  extends MetroscopiaNPP_direct(superheater(faulty=true), LP_reheater(faulty=true), HP_reheater(faulty=true));

  Real Failure_superheater_fouling(start=0);
  Real Failure_superheater_closed_vent(start=0);

  Real Failure_LP_reheater_fouling(start=0);

  Real Failure_HP_reheater_fouling(start=0);
  Real Failure_water_level_rise(start=0);
equation
  superheater.fouling = Failure_superheater_fouling;
  superheater.closed_vent = Failure_superheater_closed_vent;
  LP_reheater.fouling = Failure_LP_reheater_fouling;
  HP_reheater.fouling = Failure_HP_reheater_fouling;
  HP_reheater.water_level_rise = Failure_water_level_rise;

  Failure_superheater_fouling = 10*time;
  Failure_superheater_closed_vent = 100*time;
  Failure_LP_reheater_fouling = 10*time;
  Failure_HP_reheater_fouling = 10*time;
  Failure_water_level_rise = - 0.1*time;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MetroscopiaNPP_faulty;
