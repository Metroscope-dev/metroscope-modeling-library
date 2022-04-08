within MetroscopeModelingLibrary.Fuel;
package BaseClasses
  model FuelFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FuelBaseClassIcon;
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends Partial.BaseClasses.FlowModel(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelInlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelOutlet C_out,
      redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));

    import MetroscopeModelingLibrary.Units.Inputs;
    Inputs.InputPower W_input(start=0);
    Inputs.InputDifferentialPressure DP_input(start=0);
  equation
    W = W_input;
    DP = DP_input;
  end FuelFlowModel;

  model FuelIsoPFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FuelBaseClassIcon;
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends Partial.BaseClasses.IsoPFlowModel(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelInlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelOutlet C_out,
      redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));

    import MetroscopeModelingLibrary.Units.Inputs;
    Inputs.InputPower W_input(start=0);
  equation
    W = W_input;
  end FuelIsoPFlowModel;

  model FuelIsoHFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FuelBaseClassIcon;
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends Partial.BaseClasses.IsoHFlowModel(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelInlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelOutlet C_out,
      redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));

    import MetroscopeModelingLibrary.Units.Inputs;
    Inputs.InputDifferentialPressure DP_input(start=0);
  equation
    DP = DP_input;
  end FuelIsoHFlowModel;

  model FuelIsoPHFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FuelBaseClassIcon;
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends Partial.BaseClasses.IsoPHFlowModel(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelInlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelOutlet C_out,
      redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
  end FuelIsoPHFlowModel;
  annotation (Icon(graphics={
      Rectangle(
        lineColor={200,200,200},
        fillColor={248,248,248},
        fillPattern=FillPattern.HorizontalCylinder,
        extent={{-100,-100},{100,100}},
        radius=25.0),
      Rectangle(
        lineColor={128,128,128},
        extent={{-100,-100},{100,100}},
        radius=25.0),
   Rectangle(
        extent={{-46,49},{46,-47}},
        lineColor={213,213,0},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid,
        lineThickness=1),
      Rectangle(
        extent={{26,18},{60,-16}},
        lineColor={213,213,0},
        lineThickness=1,
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
      Rectangle(
        extent={{-64,19},{-28,-17}},
        lineColor={213,213,0},
        lineThickness=1,
        fillColor={213,213,0},
        fillPattern=FillPattern.Solid)}));
end BaseClasses;
