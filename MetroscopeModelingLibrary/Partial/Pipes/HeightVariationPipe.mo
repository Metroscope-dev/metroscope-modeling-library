within MetroscopeModelingLibrary.Partial.Pipes;
partial model HeightVariationPipe
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoHFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Constants;

  parameter Units.Height delta_z_constant = 10;
  Utilities.Interfaces.GenericReal delta_z(start=delta_z_constant, nominal=delta_z_constant) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));
equation
  DP = - rho_in*Constants.g*delta_z;
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
end HeightVariationPipe;
