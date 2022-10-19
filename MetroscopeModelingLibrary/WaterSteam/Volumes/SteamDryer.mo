within MetroscopeModelingLibrary.WaterSteam.Volumes;
model SteamDryer
  extends Partial.Volumes.PhaseSeparationVolume;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputMassFraction x_steam_outlet(start=0.99);
equation
  x_steam_out = x_steam_outlet;
end SteamDryer;
