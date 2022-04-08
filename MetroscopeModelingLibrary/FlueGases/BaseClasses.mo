within MetroscopeModelingLibrary.FlueGases;
package BaseClasses
  model FlueGasesFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FlueGasesBaseClassIcon;
    package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
    extends Partial.BaseClasses.FlowModel(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
      redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));

    import MetroscopeModelingLibrary.Units.Inputs;
    Inputs.InputPower W_input(start=0);
    Inputs.InputDifferentialPressure DP_input(start=0);
  equation
    W = W_input;
    DP = DP_input;
  end FlueGasesFlowModel;

  model FlueGasesIsoPFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FlueGasesBaseClassIcon;
    package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
    extends Partial.BaseClasses.IsoPFlowModel(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
      redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));

    import MetroscopeModelingLibrary.Units.Inputs;
    Inputs.InputPower W_input(start=0);
  equation
    W = W_input;
  end FlueGasesIsoPFlowModel;

  model FlueGasesIsoHFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FlueGasesBaseClassIcon;
    package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
    extends Partial.BaseClasses.IsoHFlowModel(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
      redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));

    import MetroscopeModelingLibrary.Units.Inputs;
    Inputs.InputDifferentialPressure DP_input(start=0);
  equation
    DP = DP_input;
  end FlueGasesIsoHFlowModel;

  model FlueGasesIsoPHFlowModel
    extends MetroscopeModelingLibrary.Icons.BaseClasses.FlueGasesBaseClassIcon;
    package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
    extends Partial.BaseClasses.IsoPHFlowModel(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
      redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));
  end FlueGasesIsoPHFlowModel;
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
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Rectangle(
          extent={{26,18},{60,-16}},
          lineColor={175,175,175},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-64,19},{-28,-17}},
          lineColor={175,175,175},
          lineThickness=1,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid)}));
end BaseClasses;
