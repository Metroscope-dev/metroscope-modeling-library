within MetroscopeModelingLibrary.Common.Constants;
partial model Constants
public
  parameter Real eps=1e-3 "Small number for pressure loss equation";
  parameter Modelica.SIunits.MassFlowRate Qeps=1.e-3
    "Small mass flow for continuous flow reversal";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Constants;
