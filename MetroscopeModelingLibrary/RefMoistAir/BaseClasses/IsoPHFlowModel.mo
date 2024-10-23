within MetroscopeModelingLibrary.RefMoistAir.BaseClasses;
model IsoPHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.RefMoistAirBaseClassIcon;
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));

import MetroscopeModelingLibrary.Utilities.Units;

  // Indicators related to water content in moist air
  Units.MassFraction x_liq_in; // Liquid water mass fraction at the inlet
  Units.MassFraction x_liq_out; // Liquid water mass fraction at the outlet
  Real relative_humidity_in; // Relative humidity at the inlet
  Real relative_humidity_out; // Relative humidity at the outlet

equation
  // Indicators
  x_liq_in = Medium.massFractionWaterNonVapor(state_in);
  x_liq_out = Medium.massFractionWaterNonVapor(state_out);
  relative_humidity_in = Medium.relativeHumidity(state_in);
  relative_humidity_out = Medium.relativeHumidity(state_out);

end IsoPHFlowModel;
