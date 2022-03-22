within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model FlowModel "PartialTransportModel with no flow or species variation"
  extends PartialTransport.PartialTransportModel(Q_in_0 = Q_0, Q_out_0 = Q_0);
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.MassFlowRate Q_0 = 100;

  // Input Quantity
  Units.MassFlowRate Q(start=Q_0) "Component mass flow rate";
equation
  // Input Quantity
  Q = Q_in;

  // Conservation equations
  DM = 0;
  DXi = zeros(Medium.nXi);
end FlowModel;
