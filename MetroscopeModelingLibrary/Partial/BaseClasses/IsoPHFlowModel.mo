within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model IsoPHFlowModel "FlowModel with no pressure or enthalpy variations"
  extends FlowModel(Qv_in_0=Qv_0, Qv_out_0=-Qv_0, P_in_0=P_0, P_out_0=P_0, h_in_0=h_0, h_out_0=h_0, T_in_0=T_0, T_out_0=T_0) annotation(IconMap(primitivesVisible=primitivesVisible));
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.InletVolumeFlowRate Qv_0 = 100;
  parameter Units.Pressure P_0 = 1e5;
  parameter Units.Temperature T_0 = 300;

  // Input Quantities
  Units.InletVolumeFlowRate Qv(start=Qv_0, nominal=Qv_0) "Component volume flow rate"; // No volume flow rate variation in IsoPHFlowModel
  Units.SpecificEnthalpy h "Enthalpy of the fluid into the component";
  Units.Pressure P(start=P_0) "Pressure of the fluid into the component";
equation
  // Input Quantities
  P = P_in;
  h = h_in;
  Qv = Qv_in;

  // Conservation equation
  DP = 0;
  W = 0;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end IsoPHFlowModel;
