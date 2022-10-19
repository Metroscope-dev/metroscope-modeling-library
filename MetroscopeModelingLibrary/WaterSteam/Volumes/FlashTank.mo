within MetroscopeModelingLibrary.WaterSteam.Volumes;
model FlashTank
  extends Partial.Volumes.PhaseSeparationVolume;
equation
  x_steam_out = 1;
end FlashTank;
