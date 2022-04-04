within MetroscopeModelingLibrary.Power.Connectors;
partial connector PowerPort
  import MetroscopeModelingLibrary.Units;

  replaceable flow Units.Power W constrainedby Units.Power;
  Real unused;
end PowerPort;
