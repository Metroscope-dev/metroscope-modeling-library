within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model IsoHFlowModel
  extends FlowModel(T_in_0=T_0, T_out_0=T_0);
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.Temperature T_0 = 300;

  // Input Quantity
  Units.SpecificEnthalpy h "Enthalpy of the fluid into the component";
equation
  // Input Quantity
  h = h_in;
  h_in = h_out;

  // Conservation equation
  //W = 0;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end IsoHFlowModel;
