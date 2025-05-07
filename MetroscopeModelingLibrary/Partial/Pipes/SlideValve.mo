within MetroscopeModelingLibrary.Partial.Pipes;
partial model SlideValve
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoHFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;


  parameter Boolean faulty = false;
  Units.Percentage closed_valve; // Valve not fully opened

  parameter Real Cv_constant = 1e4;

  Utilities.Interfaces.GenericReal Cv(start=Cv_constant) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-40,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-40,110})));
equation
    // Failure modes
  if not faulty then
    closed_valve = 0;
  end if;

  /* Pressure loss */
  DP*(1 - closed_valve/100)^2*Cv*abs(Cv) = -1.733e12*abs(Q)*Q/rho_in^2;

  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-40},{100,180}},
        grid={2,2}), graphics={
        Polygon(
          points={{40,102},{-40,102},{-40,118},{-38,136},{-32,146},{-20,156},{0,
              162},{20,156},{32,146},{38,134},{40,116},{40,102}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{40,102},{-40,102},{0,2}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-100,-40},{0,2},{-100,42},{-100,-40},{-100,-40}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{100,42},{100,-40},{0,2},{0,2}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),
    Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-40},{100,180}},
        grid={2,2}), graphics={
        Polygon(
          points={{-100,-40},{0,2},{-100,42},{-100,-40},{-100,-40}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{100,42},{100,-40},{0,2},{0,2}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{40,102},{-40,102},{0,2}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{40,102},{-40,102},{-40,118},{-38,136},{-32,146},{-20,156},{0,162},{20,156},{32,146},{38,134},{40,116},{40,102}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end SlideValve;
