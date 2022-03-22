within MetroscopeModelingLibrary.Partial.BaseClasses;
model IsoPHFlowModel "FlowModel with no pressure or enthalpy variations"
  extends FlowModel(P_in_0 = P_0, P_out_0 = P_0, h_in_0 = h_0, h_out_0 = h_0);
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.SpecificEnthalpy h_0 = 1e5;
  parameter Units.Pressure P_0 = 1e5;

  // Input Quantities
  Units.SpecificEnthalpy h(start=h_0) "Enthalpy of the fluid into the component";
  Units.Pressure P(start=P_0) "Pressure of the fluid into the component";
equation
  // Input Quantities
  P = P_in;
  h = h_in;

  // Conservation equation
  DP = 0;
  W = 0;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end IsoPHFlowModel;
