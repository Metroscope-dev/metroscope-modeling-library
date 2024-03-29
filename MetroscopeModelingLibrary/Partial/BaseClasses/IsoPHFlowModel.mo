within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model IsoPHFlowModel "FlowModel with no pressure or enthalpy variations"
  extends FlowModel(P_in_0=P_0, P_out_0=P_0, T_in_0=T_0, T_out_0=T_0, h_in_0=h_0, h_out_0=h_0) annotation(IconMap(primitivesVisible=primitivesVisible));
  import MetroscopeModelingLibrary.Utilities.Units;

  // Initialization parameters
  parameter Units.Pressure P_0 = 1e5;
  parameter Units.Temperature T_0 = 300;
  parameter Units.SpecificEnthalpy h_0=5e5;

  // Input Quantities
  Units.SpecificEnthalpy h(start=h_0) "Enthalpy of the fluid into the component";
  Units.Pressure P(start=P_0) "Pressure of the fluid into the component";
  Units.Temperature T(start=T_0) "Temperature of the fluid into the component";

equation
  // Input Quantities
  P = P_in;
  h = h_in;
  T = T_in;

  // Conservation equation
  DP = 0;
  DH = 0;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end IsoPHFlowModel;
