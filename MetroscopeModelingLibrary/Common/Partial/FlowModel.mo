within MetroscopeModelingLibrary.Common.Partial;
model FlowModel
  extends BasicTransportModel(Q_in_0 = Q_0);
  parameter Real Q_0 = 100;
  connector InputMassFlowRate = input Modelica.Units.SI.MassFlowRate;
  InputMassFlowRate Q(start=100) "Inlet Mass flow rate";
equation
  DM = 0;
  DXi = zeros(Medium.nXi);
  Q = Q_in;
end FlowModel;
