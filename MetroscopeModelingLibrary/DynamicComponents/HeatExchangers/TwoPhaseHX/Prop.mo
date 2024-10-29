within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX;
record Prop
  import MetroscopeModelingLibrary.Utilities.Units;
    Units.Temperature T "Temperature";
  Units.Density d "Density";
  Modelica.Units.SI.InternalEnergy u "Specific inner energy";
  Modelica.Units.SI.SpecificEntropy s "Specific entropy";
  Real cp "Specific heat capacity at constant presure";
  Real ddhp "Derivative of density wrt. specific enthalpy at constant pressure";
  Real ddph "Derivative of density wrt. pressure at constant specific enthalpy";
  Real duph(unit="m3/kg") "Derivative of specific inner energy wrt. pressure at constant specific enthalpy";
  Real duhp(unit = "1") "Derivative of specific inner energy wrt. specific enthalpy at constant pressure";
  ThermoSysPro.Units.SI.MassFraction x "Vapor mass fraction";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Prop;
