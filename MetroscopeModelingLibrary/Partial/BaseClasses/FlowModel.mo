within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model FlowModel "PartialTransportModel with no flow or species variation"
  extends PartialTransportModel(Q_in_0=Q_0, Q_out_0=Q_0, Xi_in_0=Xi_0, Xi_out_0=Xi_0) annotation(IconMap(primitivesVisible=true));
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.MassFlowRate Q_0 = 100;
  parameter Units.MassFraction Xi_0[Medium.nXi] = zeros(Medium.nXi);

  // Input Quantity
  Units.MassFlowRate Q(start=Q_0) "Component mass flow rate";
  Units.MassFraction Xi[Medium.nXi](start=Xi_0) "Component mass fractions";
equation
  // Input Quantity
  Q = Q_in;
  Xi = Xi_in;

  // Conservation equations
  DM = 0;
  DXi = zeros(Medium.nXi);
end FlowModel;
