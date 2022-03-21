within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model IsoHFlowModel
  extends FlowModel(h_in_0 = h_0, h_out_0 = h_0);
  import MetroscopeModelingLibrary.Connectors.InputConnectors;
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.SpecificEnthalpy h_0 = 1e5;

  // Input Quantity
  Units.SpecificEnthalpy h(start=h_0) "Enthalpy of the fluid into the component";
equation
  // Input Quantity
  h = h_in;

  // Conservation equation
  W = 0;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end IsoHFlowModel;
