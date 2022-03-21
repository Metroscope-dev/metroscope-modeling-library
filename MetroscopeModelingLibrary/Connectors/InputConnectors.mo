within MetroscopeModelingLibrary.Connectors;
package InputConnectors
  import MetroscopeModelingLibrary.Units;

  connector InputAngularVelocity = input Units.AngularVelocity;
  connector InputArea = input Units.Area;
  connector InputCv = input Units.Cv;
  connector InputDifferentialPressure = input Units.DifferentialPressure;
  connector InputDifferentialTemperature = input Units.DifferentialTemperature;
  connector InputFrictionCoefficient = input Units.FrictionCoefficient;
  connector InputHeight =input Units.Height;
  connector InputMassFlowRate = input Units.MassFlowRate;
  connector InputMassFraction = input Units.MassFraction;
  connector InputPower = input Units.Power;
  connector InputPressure = input Units.Pressure;
  connector InputSpecificEnthalpy = input Units.SpecificEnthalpy;
  connector InputReal = input Real;
  connector InputVelocity = input Units.Velocity;
  connector InputTemperature = input Units.Temperature;
  connector InputYield = input Units.Yield;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          extent={{-74,26},{-24,-24}},
          lineColor={0,140,72},
          lineThickness=1),
        Line(
          points={{-24,0},{24,0}},
          color={0,140,72},
          thickness=1),
        Rectangle(
          extent={{24,30},{82,-28}},
          lineColor={0,140,72},
          lineThickness=1,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}));
end InputConnectors;
