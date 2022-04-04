within MetroscopeModelingLibrary.Power.Connectors;
connector PowerInlet
  extends Icons.Connectors.PowerInletIcon;
  import MetroscopeModelingLibrary.Units;

  extends MetroscopeModelingLibrary.Power.Connectors.PowerPort(redeclare flow Units.InletPower W);
end PowerInlet;
