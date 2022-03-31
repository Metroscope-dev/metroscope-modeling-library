within MetroscopeModelingLibrary.Icons;
partial model ModelColors
  extends MetroscopeModelingLibrary.Icons.Colors;
equation
  assert(medium_name <> "", "You must assign a value to your medium_name parameter, among 'Partial', 'WaterSteam', 'MoistAir'");
end ModelColors;
