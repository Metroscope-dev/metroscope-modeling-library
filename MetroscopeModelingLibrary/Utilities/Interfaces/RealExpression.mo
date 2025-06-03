within MetroscopeModelingLibrary.Utilities.Interfaces;
block RealExpression "Set output signal to a time varying Real expression"

  Modelica.Blocks.Interfaces.RealOutput y=0.0 "Value of Real output"
    annotation (Dialog(group="Time varying output signal"), Placement(
        transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-50}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-50})));

  annotation (Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}}), graphics={
        Rectangle(
          extent={{-100,40},{100,-40}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Text(
          extent={{-96,15},{96,-15}},
          textColor={0,0,0},
          textString="%y"),
        Text(
          extent={{-100,80},{100,40}},
          textColor={0,0,0},
          textString="%name")}),  Documentation(info="<html>
<p>
The (time varying) Real output signal of this block can be defined in its
parameter menu via variable <strong>y</strong>. The purpose is to support the
easy definition of Real expressions in a block diagram. For example,
in the y-menu the definition \"if time &lt; 1 then 0 else 1\" can be given in order
to define that the output signal is one, if time &ge; 1 and otherwise
it is zero. Note, that \"time\" is a built-in variable that is always
accessible and represents the \"model time\" and that
variable <strong>y</strong> is both a variable and a connector.
</p>
</html>"));

end RealExpression;
