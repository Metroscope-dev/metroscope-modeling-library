within MetroscopeModelingLibrary.Common.Partial;
model IsoPFlowModel
  extends FlowModel;
  parameter Real Q_0 = Q_in_0;
  parameter Real P_0 = P_in_0;
equation
  DP = 0;
end IsoPFlowModel;
