within MetroscopeModelingLibrary.Utilities;
package Units
  extends Modelica.Icons.Package;
   import Modelica.Units.SI;

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

  type AngularVelocity = SI.AngularVelocity;
  type Area = SI.Area(nominal=100);
  type AtomicMass = SI.RelativeAtomicMass(min=0);
  type MolecularMass = SI.RelativeMolecularMass(min=0);
  type Cv = Real(final quantity="Cv U.S.", final unit="m4/(s.N5)");
  type Cst = Real(final quantity="Stodola constant", min=0, nominal=1e4, start=1e4);
  type Density = SI.Density(start=1);
  type DifferentialPressure = SI.PressureDifference(nominal=1.0e5, start=1.0e5, min=-1.0e8, max=1.0e8);
  type DifferentialTemperature = SI.TemperatureDifference(nominal=100, start=0, min=-2000, max=2000);
  type DifferentialEnthalpy = Real(nominal=1e4, start=0, min=-2e7, max=2e7);
  type FrictionCoefficient = Real(quantity="FrictionCoefficient", unit="m-4", nominal=1e-3);
  type Fraction = SI.PerUnit(min=0., max=1., nominal=0.5);
  type Percentage = Real(min=0., max=100., nominal=50);
  type GasDensity = Density(start=5, nominal = 5) "start value for gases/vapours";
  type HeatExchangeCoefficient = SI.CoefficientOfHeatTransfer(start=1e3, nominal= 1e3);
  type HeatCapacity = SI.SpecificHeatCapacity;
  type Height = SI.Position(nominal=1);
  type DifferentialHeight = SI.Length(nominal=1);
  type LiquidDensity = Density(start=1000, nominal = 1000) "start value for liquids";
  type MassFlowRate = SI.MassFlowRate(nominal=1e3);
  type PositiveMassFlowRate = SI.MassFlowRate(min=0, start=1e3, nominal=1e3);
  type NegativeMassFlowRate=SI.MassFlowRate(max=0, start=-1e3, nominal=-1e3);
  type MassFraction = SI.MassFraction;
  type Temperature = SI.Temperature(start=300, nominal=300) "in K";
  type Power = SI.Power(displayUnit="MW");
  type PositivePower =
                    SI.Power(min=0, nominal=1e5, start=1e5, displayUnit="MW");
  type NegativePower=SI.Power(max=0, nominal=-1e5, start=-1e5, displayUnit="MW");
  type Pressure = SI.AbsolutePressure "Absolute pressure, in Pa";
  type SpecificEnthalpy = SI.SpecificEnthalpy(start=5e5, nominal=5e5);
  type Velocity = SI.Velocity;
  type VolumeFlowRate = SI.VolumeFlowRate(start=0.1, nominal=0.1);
  type PositiveVolumeFlowRate =
                             VolumeFlowRate(min=0, nominal=0.1, start=0.1);
  type NegativeVolumeFlowRate=VolumeFlowRate(max=0, start=-0.1, nominal=-0.1);
  type Yield = Fraction;
  annotation(Icon(graphics={
      Polygon(
        fillColor = {128,128,128},
        pattern = LinePattern.None,
        fillPattern = FillPattern.Solid,
        points = {{-80,-40},{-80,-40},{-55,50},{-52.5,62.5},{-65,60},{-65,65},{-35,77.5},{-32.5,60},{-50,0},{-50,0},{-30,15},{-20,27.5},{-32.5,27.5},{-32.5,27.5},{-32.5,32.5},{-32.5,32.5},{2.5,32.5},{2.5,32.5},{2.5,27.5},{2.5,27.5},{-7.5,27.5},{-30,7.5},{-30,7.5},{-25,-25},{-17.5,-28.75},{-10,-25},{-5,-26.25},{-5,-32.5},{-16.25,-41.25},{-31.25,-43.75},{-40,-33.75},{-45,-5},{-45,-5},{-52.5,-10},{-52.5,-10},{-60,-40},{-60,-40}},
        smooth = Smooth.Bezier),
      Polygon(
        fillColor = {128,128,128},
        pattern = LinePattern.None,
        fillPattern = FillPattern.Solid,
        points = {{87.5,30},{62.5,30},{62.5,30},{55,33.75},{36.25,35},{16.25,25},{7.5,6.25},{11.25,-7.5},{22.5,-12.5},{22.5,-12.5},{6.25,-22.5},{6.25,-35},{16.25,-38.75},{16.25,-38.75},{21.25,-41.25},{21.25,-41.25},{45,-48.75},{47.5,-61.25},{32.5,-70},{12.5,-65},{7.5,-51.25},{21.25,-41.25},{21.25,-41.25},{16.25,-38.75},{16.25,-38.75},{6.25,-41.25},{-6.25,-50},{-3.75,-68.75},{30,-76.25},{65,-62.5},{63.75,-35},{27.5,-26.25},{22.5,-20},{27.5,-15},{27.5,-15},{30,-7.5},{30,-7.5},{27.5,-2.5},{28.75,11.25},{36.25,27.5},{47.5,30},{53.75,22.5},{51.25,8.75},{45,-6.25},{35,-11.25},{30,-7.5},{30,-7.5},{27.5,-15},{27.5,-15},{43.75,-16.25},{65,-6.25},{72.5,10},{70,20},{70,20},{80,20}},
        smooth = Smooth.Bezier)}));
end Units;
