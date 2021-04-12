within MetroscopeModelingLibrary.Common.Functions;
function ConditionalEnthalpyPropagation
  input Real Q;
  input Real Qeps;
  input Real h_negative_flow;
  input Real h_positive_flow;
  output Real h;
algorithm
  if Q >= 0 then
    h := h_positive_flow;
  else
    h := h_negative_flow;
  end if;
  annotation (smoothOrder = 1,
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics),
    Window(
      x=0.11,
      y=0.2,
      width=0.6,
      height=0.6),
    Documentation(info="<html>
<p><b></b></p>
</HTML>
<html>
<p><b>ThermoSysPro Version 2.0</b></p>
</HTML>
"));
end ConditionalEnthalpyPropagation;
