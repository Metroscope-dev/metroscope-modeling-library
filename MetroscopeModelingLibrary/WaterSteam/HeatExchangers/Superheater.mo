within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Superheater
  extends Superheater_PartialCondensation;

equation
  x_hot_out = 0;  //Total condensation
end Superheater;
