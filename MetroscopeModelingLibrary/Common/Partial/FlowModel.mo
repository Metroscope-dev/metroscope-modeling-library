within MetroscopeModelingLibrary.Common.Partial;
model FlowModel
  extends BasicTransportModel;
  connector InputMassFlowRate = input Modelica.Units.SI.MassFlowRate;
  InputMassFlowRate Q(start=Q_in_0) "Inlet Mass flow rate";
equation
  DM = 0;
  DXi = zeros(Medium.nXi);
  Q = Q_in;
end FlowModel;
