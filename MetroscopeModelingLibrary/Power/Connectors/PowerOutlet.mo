within MetroscopeModelingLibrary.Power.Connectors;
connector PowerOutlet
  extends Icons.Connectors.PowerOutletIcon;
  import MetroscopeModelingLibrary.Units;

  extends MetroscopeModelingLibrary.Power.Connectors.PowerPort(redeclare flow Units.OutletPower W);
end PowerOutlet;
