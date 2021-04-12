within MetroscopeModelingLibrary.WaterSteam.Functions;
function VaporMassFraction
      replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.Functions.VaporMassFraction( redeclare
      package Medium =
        WaterSteamMedium);
end VaporMassFraction;
