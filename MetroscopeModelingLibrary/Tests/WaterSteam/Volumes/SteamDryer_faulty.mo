within MetroscopeModelingLibrary.Tests.WaterSteam.Volumes;
model SteamDryer_faulty
  extends SteamDryer     (steamDryer( faulty = true));
  input Real Fault_SteamDryer_Eff_Decrease( start = 0);

equation
  steamDryer.MS_eff_decrease = Fault_SteamDryer_Eff_Decrease + 0.1*time;

end SteamDryer_faulty;
