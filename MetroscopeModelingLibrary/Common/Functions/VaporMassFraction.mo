within MetroscopeModelingLibrary.Common.Functions;
function VaporMassFraction
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialTwoPhaseMedium;
  input Modelica.SIunits.AbsolutePressure P;
  input Modelica.SIunits.SpecificEnthalpy h;
  output Real x;
protected
  Modelica.SIunits.SpecificEnthalpy hl;
  Modelica.SIunits.SpecificEnthalpy hv;
algorithm
  hv :=Medium.dewEnthalpy(Medium.setSat_p(P));
  hl :=Medium.bubbleEnthalpy(Medium.setSat_p(P));
  if h < hv and h > hl then
    x := (h-hl)/(hv-hl);
  elseif h < hl then
    x := 0;
  else
    x := 1;
  end if;
end VaporMassFraction;
