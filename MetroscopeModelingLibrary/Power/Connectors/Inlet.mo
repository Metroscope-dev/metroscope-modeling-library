within MetroscopeModelingLibrary.Power.Connectors;
connector Inlet
  extends Icons.Connectors.PowerInletIcon;
  import MetroscopeModelingLibrary.Units;

  flow Units.PositivePower W;
  Units.NotUsed not_used "not used effort variable to balance the connector";
end Inlet;
