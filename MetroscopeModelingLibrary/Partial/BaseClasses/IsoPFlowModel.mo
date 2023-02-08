within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model IsoPFlowModel "Flow model with no pressure variation"
  extends FlowModel(P_in_0=P_0, P_out_0=P_0);
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Initialization parameters
  parameter Units.Pressure P_0 = 1e5;

  // Input Quantity
  Units.Pressure P(start=P_0) "Pressure of the fluid into the component";
equation
  // Input Quantity
  P = P_in;

  // Conservation equation
  DP = 0;
end IsoPFlowModel;
