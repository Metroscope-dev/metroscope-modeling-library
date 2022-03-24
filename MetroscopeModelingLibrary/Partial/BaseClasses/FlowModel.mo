within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model FlowModel "PartialTransportModel with no flow or species variation"
  extends PartialTransportModel(Q_in_0=Q_0, Q_out_0=Q_0) annotation(IconMap(primitivesVisible=true));
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Initialization parameters
  parameter Units.MassFlowRate Q_0 = 100;
  parameter Units.MassFraction Xi_0 = 100;

  // Input Quantity
  Units.MassFlowRate Q(start=Q_0) "Component mass flow rate";
  Units.MassFraction Xi[Medium.nXi] = zeros(Medium.nXi) "Component mass fractions";
equation
  // Input Quantity
  Q = Q_in;
  Xi = Xi_in;

  // Conservation equations
  DM = 0;
  DXi = zeros(Medium.nXi);
end FlowModel;
