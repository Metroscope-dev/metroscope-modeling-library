within MetroscopeModelingLibrary.Common.Functions;
function ThermoSquare "Thermodynamic square"
  extends Modelica.Icons.Function;
  input Real x;
  input Real dx;
  output Real y;
algorithm
  y := if (abs(x) > dx) then x*abs(x) else x*dx;
  annotation (smoothOrder = 1,
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics),
    Documentation(info="<html>
<p><b>Copyright &copy; EDF 2002 - 2010</b></p>
</HTML>
<html>
<p><b>ThermoSysPro Version 2.0</b></p>
</HTML>
"));
end ThermoSquare;
