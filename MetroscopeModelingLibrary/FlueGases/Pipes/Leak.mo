within MetroscopeModelingLibrary.FlueGases.Pipes;
model Leak

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.Pipes.Leak(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoHFlowModel flow_model,
    redeclare MetroscopeModelingLibrary.Sensors.FlueGases.FlowSensor flow_sensor,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));

 annotation (Icon(graphics={Rectangle(
          extent={{-100,40},{0,-40}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
                             Rectangle(
          extent={{0,40},{100,-40}},
          lineColor={175,175,175},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{12,16},{36,6}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{8,0},{24,-6}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,2},{60,-6}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,10},{80,2}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,-6},{84,-14}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,-12},{42,-20}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid)}));
end Leak;
