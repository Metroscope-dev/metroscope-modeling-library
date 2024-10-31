within MetroscopeModelingLibrary.RefMoistAir.Pipes;
model Leak

  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.Pipes.Leak(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoHFlowModel flow_model,
    redeclare MetroscopeModelingLibrary.Sensors.RefMoistAir.FlowSensor flow_sensor,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
                            Rectangle(
          extent={{-100,40},{0,-40}},
          lineColor={0,127,127},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid),
                             Rectangle(
          extent={{0,40},{100,-40}},
          lineColor={0,127,127},
          fillColor={0,160,160},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{12,16},{36,6}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{8,0},{24,-6}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{36,2},{60,-6}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{56,10},{80,2}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{60,-6},{84,-14}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{18,-12},{42,-20}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end Leak;
