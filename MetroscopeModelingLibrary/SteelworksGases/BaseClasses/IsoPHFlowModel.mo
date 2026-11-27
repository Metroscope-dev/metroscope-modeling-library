within MetroscopeModelingLibrary.SteelworksGases.BaseClasses;
model IsoPHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.SteelworksBaseClassIcon;
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Outlet C_out,
    redeclare package Medium = SteelworksGasMedium) annotation (IconMap(primitivesVisible=false));
end IsoPHFlowModel;
