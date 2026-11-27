within MetroscopeModelingLibrary.SteelworksGases.Pipes;
model Leak

  model SteelworksFlowSensor
    extends Partial.Sensors.FlowSensor(
      redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.SteelworksGases.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium);
  end SteelworksFlowSensor;

  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  extends Partial.Pipes.Leak(
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Outlet C_out,
    redeclare
      MetroscopeModelingLibrary.SteelworksGases.BaseClasses.IsoHFlowModel flow_model,
    redeclare SteelworksFlowSensor flow_sensor,
    redeclare package Medium = SteelworksGasMedium) annotation (IconMap(primitivesVisible=false));

 annotation (Icon(graphics={Rectangle(
          extent={{-100,40},{0,-40}},
          lineColor={95,95,95},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid),
                             Rectangle(
          extent={{0,40},{100,-40}},
          lineColor={175,175,175},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{12,16},{36,6}},
          lineColor={95,95,95},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{8,0},{24,-6}},
          lineColor={95,95,95},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,2},{60,-6}},
          lineColor={95,95,95},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,10},{80,2}},
          lineColor={95,95,95},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,-6},{84,-14}},
          lineColor={95,95,95},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,-12},{42,-20}},
          lineColor={95,95,95},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid)}));
end Leak;
