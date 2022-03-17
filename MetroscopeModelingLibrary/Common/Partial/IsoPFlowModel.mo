within MetroscopeModelingLibrary.Common.Partial;
model IsoPFlowModel
  extends FlowModel(P_in_0 = P_0);
  parameter Real P_0 = 10e5;
  connector InputPressure = input Modelica.Units.SI.AbsolutePressure;
  InputPressure P(start=1e5) "Inlet Mass flow rate";
equation
  DP = 0;
  P = P_in;
end IsoPFlowModel;
