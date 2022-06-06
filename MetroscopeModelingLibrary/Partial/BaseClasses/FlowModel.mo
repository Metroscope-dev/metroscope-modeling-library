within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model FlowModel "PartialTransportModel with no flow or species variation"
  extends MetroscopeModelingLibrary.Icons.BaseClasses.BaseClassIcon;

  extends PartialTransportModel(Q_in(start=Q_0), Q_out(start=-Q_0), C_in(Q(start=Q_0)), C_out(Q(start=-Q_0))) annotation(IconMap(primitivesVisible=true));
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.PositiveMassFlowRate Q_0=100;

  // Input Quantity
  Units.PositiveMassFlowRate Q(start=Q_0, nominal=Q_0) "Component mass flow rate";
  Units.MassFraction Xi[Medium.nXi] "Component mass fractions";
equation
  // Input Quantity
  Q = Q_in;
  Xi = Xi_in;

  // Conservation equations
  DM = 0;
  DXi = zeros(Medium.nXi);
end FlowModel;
