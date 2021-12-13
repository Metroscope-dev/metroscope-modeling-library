within MetroscopeModelingLibrary.Common.Functions;
function SigmoidEnthalpyPropagation
  input Real Q;
  input Real Qeps;
  input Real h_negative_flow;
  input Real h_positive_flow;
  output Real h;
algorithm
  h := h_negative_flow + (1/(1+exp(-Q/Qeps)))*(h_positive_flow-h_negative_flow);
  annotation (smoothOrder = 1,
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics),
    Documentation(info="<html>
<p><b></b></p>
</HTML>
<html>
<p><b>ThermoSysPro Version 2.0</b></p>
</HTML>
"));
end SigmoidEnthalpyPropagation;
