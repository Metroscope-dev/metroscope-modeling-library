within MetroscopeModelingLibrary.MoistAir.Pipes;
model Leak

  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.Pipes.Leak(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoHFlowModel flow_model,
    redeclare MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor flow_sensor,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
                            Rectangle(
          extent={{-100,40},{0,-40}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
                             Rectangle(
          extent={{0,40},{100,-40}},
          lineColor={85,170,255},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{12,16},{36,6}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{8,0},{24,-6}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,2},{60,-6}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,10},{80,2}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,-6},{84,-14}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,-12},{42,-20}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid)}));
end Leak;
