within MetroscopeModelingLibrary.Utilities.Units;
package Inputs
  import MetroscopeModelingLibrary.Utilities.Units;

  connector InputAngularVelocity = input Units.AngularVelocity;
  connector InputArea = input Units.Area;
  connector InputCv = input Units.Cv;
  connector InputCst = input Units.Cst;
  connector InputDifferentialPressure = input Units.DifferentialPressure;
  connector InputDifferentialTemperature = input Units.DifferentialTemperature;
  connector InputFraction = input Units.Fraction;
  connector InputPercentage = input Units.Percentage;
  connector InputFrictionCoefficient = input Units.FrictionCoefficient;
  connector InputHeatExchangeCoefficient = input Units.HeatExchangeCoefficient;
  connector InputHeatCapacity = input Units.HeatCapacity;
  connector InputHeight =input Units.Height;
  connector InputDifferentialHeight =input Units.DifferentialHeight;
  connector InputMassFlowRate = input Units.MassFlowRate;
  connector InputPositiveMassFlowRate = input Units.PositiveMassFlowRate;
  connector InputNegativeMassFlowRate=input Units.NegativeMassFlowRate;
  connector InputMassFraction = input Units.MassFraction;
  connector InputPower = input Units.Power;
  connector InputPositivePower = input Units.PositivePower;
  connector InputNegativePower=input Units.NegativePower;
  connector InputPressure = input Units.Pressure;
  connector InputSpecificEnthalpy = input Units.SpecificEnthalpy;
  connector InputReal = input Real;
  connector InputVelocity = input Units.Velocity;
  connector InputVolumeFlowRate = input VolumeFlowRate;
  connector InputPositiveVolumeFlowRate = input VolumeFlowRate;
  connector InputNegativeVolumeFlowRate=input VolumeFlowRate;
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
        Line(
          points={{-48,0},{0,0}},
          color={0,140,72},
          thickness=1),
        Rectangle(
          extent={{0,30},{58,-28}},
          lineColor={0,140,72},
          lineThickness=1,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}));
end Inputs;
