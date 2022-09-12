within MetroscopeModelingLibrary.Power.Connectors;
connector Outlet
  extends Icons.Connectors.PowerOutletIcon;
  import MetroscopeModelingLibrary.Units;

  flow Units.NegativePower W;
  Units.NotUsed not_used "not used effort variable to balance the connector";
end Outlet;
