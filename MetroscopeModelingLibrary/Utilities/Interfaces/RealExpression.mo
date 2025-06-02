within MetroscopeModelingLibrary.Utilities.Interfaces;
block RealExpression "Set output signal to a time varying Real expression"

  Modelica.Blocks.Interfaces.RealOutput y=0.0 "Value of Real output"
    annotation (Dialog(group="Time varying output signal"), Placement(
        transformation(extent={{100,-10},{120,10}}), iconTransformation(
        extent={{-14,-14},{14,14}},
        rotation=270,
        origin={0,-40})));

  annotation (Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}}), graphics={
        Rectangle(
          extent={{-75,32.7584},{-25,-17.2417}},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          borderPattern=BorderPattern.Raised,
          rotation=90,
          origin={6.75838,49},
          pattern=LinePattern.None,
          lineColor={226,0,0}),
        Text(
          extent={{-96,15},{96,-15}},
          textString="%y"),
        Text(
          extent={{-262,72},{262,34}},
          textColor={0,0,127},
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
