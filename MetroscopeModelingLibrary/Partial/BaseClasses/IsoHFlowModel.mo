within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model IsoHFlowModel
  extends FlowModel(T_in_0=T_0, T_out_0=T_0, h_in_0=h_0, h_out_0=h_0);
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.Temperature T_0 = 300;
  parameter Units.SpecificEnthalpy h_0 = 5e5;

  // Input Quantity
  Units.SpecificEnthalpy h(start=h_0) "Enthalpy of the fluid into the component";
equation
  // Input Quantity
  h = h_in;

  // Conservation equation
  h_in = h_out;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end IsoHFlowModel;
