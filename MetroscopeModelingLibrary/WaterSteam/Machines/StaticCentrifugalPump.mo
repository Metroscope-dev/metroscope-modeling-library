within MetroscopeModelingLibrary.WaterSteam.Machines;
model StaticCentrifugalPump
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.Machines.StaticCentrifugalPump(redeclare package
              Medium =
        WaterSteamMedium);
end StaticCentrifugalPump;
