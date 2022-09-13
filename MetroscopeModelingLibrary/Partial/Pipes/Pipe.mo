within MetroscopeModelingLibrary.Partial.Pipes;
partial model Pipe
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoHFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Constants;

  Inputs.InputFrictionCoefficient Kfr(start=10) "Friction pressure loss coefficient";
  Inputs.InputDifferentialHeight delta_z(nominal=5) "Height difference between outlet and inlet";
  Units.DifferentialPressure DP_f "Singular pressure loss";
  Units.DifferentialPressure DP_z "Singular pressure loss";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0); // Fouling percentage

equation

  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

  DP_f = - (1+ fouling/100)  * Kfr * Q^2 / rho;
  DP_z = -rho * Constants.g * delta_z;

  DP = DP_f + DP_z;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid)}),
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid), Text(
          extent={{-12,14},{16,-14}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid)}));
end Pipe;
