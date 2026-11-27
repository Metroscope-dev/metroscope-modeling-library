within MetroscopeModelingLibrary.MultiFluid.Machines;
model SteelworkGasesBurner "Steady-state combustion of steelworks gas in air"
  import Modelica.Units.SI;
  import MetroscopeModelingLibrary.Utilities.Units;
  import Data = Modelica.Media.IdealGases.Common.SingleGasesData;
  package SteelworksGasesMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  package AddedFuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  package FlueMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  parameter Boolean useAddedFuel = false "Enable the additional fuel inlet"
    annotation (Evaluate=true, Dialog(group="Fuel"));

  FlueGases.Connectors.Inlet inlet "Combustion air"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
      iconTransformation(extent={{-110,-10},{-90,10}})));
  SteelworksGases.Connectors.Inlet inlet1 "Steelworks fuel gas"
    annotation (Placement(transformation(extent={{-50,-110},{-30,-90}}),
      iconTransformation(extent={{-50,-110},{-30,-90}})));
  Fuel.Connectors.Inlet fuel_in if useAddedFuel "Additional fuel"
    annotation (Placement(transformation(extent={{40,-110},{60,-90}}),
      iconTransformation(extent={{40,-110},{60,-90}})));
  FlueGases.Connectors.Outlet outlet "Flue gases"
    annotation (Placement(transformation(extent={{90,-10},{110,10}}),
      iconTransformation(extent={{90,-10},{110,10}})));

  FlueGases.BoundaryConditions.Sink sink_air(h_in(start=h_in_air_0))
    annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  SteelworksGases.BoundaryConditions.Sink sink_fuel
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90, origin={-40,-22})));
  FlueGases.BoundaryConditions.Source source_exhaust
    annotation (Placement(transformation(extent={{12,-10},{32,10}})));
  FlueGases.Pipes.FrictionPipe pressure_loss
    annotation (Placement(transformation(extent={{46,-10},{66,10}})));

  Utilities.Interfaces.GenericReal eta(start=1, min=0, max=1)
    "Fraction of the combustion heat transferred to the flue gases"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={-40,100}),
      iconTransformation(extent={{-4,-4},{4,4}}, rotation=90, origin={-40,100})));
  Utilities.Interfaces.GenericReal Kfr "Exhaust pressure loss coefficient"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={40,100}),
      iconTransformation(extent={{-4,-4},{4,4}}, rotation=90, origin={40,100})));

  parameter Units.SpecificEnthalpy h_in_air_0 = 5e5 "Air inlet enthalpy start";
  Units.PositiveMassFlowRate Q_air;
  Units.PositiveMassFlowRate Q_fuel "Steelworks gas flow";
  Units.PositiveMassFlowRate Q_added_fuel "Additional fuel flow";
  Units.PositiveMassFlowRate Q_exhaust;
  Units.SpecificEnthalpy h_in_air(start=h_in_air_0);
  Units.SpecificEnthalpy h_in_fuel;
  Units.SpecificEnthalpy h_in_added_fuel;
  Units.SpecificEnthalpy h_exhaust;
  Units.SpecificEnthalpy HHV = steelworksHHV "Higher heating value of steelworks gas";
  Units.SpecificEnthalpy LHV = HHV - steelworksCondensationHeat "Lower heating value of steelworks gas";
  Units.SpecificEnthalpy HHV_added_fuel = addedFuelHHV "Higher heating value of the additional fuel";
  Units.SpecificEnthalpy LHV_added_fuel = HHV_added_fuel - addedFuelCondensationHeat
    "Lower heating value of the additional fuel";
  Units.Power Wth "Combustion heat transferred to the flue gases";

protected
  Fuel.BoundaryConditions.Sink sink_added_fuel
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=90, origin={50,-46})));
  Fuel.BoundaryConditions.Source idle_fuel(
    P_out=sink_air.P_in,
    Q_out=0,
    h_out=AddedFuelMedium.h_TX(298.15),
    Xi_out=AddedFuelMedium.reference_X) if not useAddedFuel
    "Valid thermodynamic state at zero flow when the external inlet is disabled"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=180, origin={84,-71})));

  // Combustible species order: CH4, H2, CO, C2H6, C3H8, n-C4H10.
  constant SI.MolarMass combustibleMolarMasses[6] =
    {Data.CH4.MM,Data.H2.MM,Data.CO.MM,Data.C2H6.MM,Data.C3H8.MM,Data.C4H10_n_butane.MM};
  constant Real carbonDioxideMoles[6] = {1,0,1,2,3,4};
  constant Real waterMoles[6] = {2,1,0,3,4,5};
  constant Real oxygenMoles[6] = {2,0.5,0.5,3.5,5,6.5};
  Units.SpecificEnthalpy steelworksHHV;
  Units.SpecificEnthalpy steelworksCondensationHeat;
  Units.SpecificEnthalpy addedFuelHHV;
  Units.SpecificEnthalpy addedFuelCondensationHeat;

  SteelworksGasesMedium.MassFraction X_fuel[SteelworksGasesMedium.nXi] "N2, CH4, CO2, H2, CO, O2, H2O";
  AddedFuelMedium.MassFraction X_added_fuel[AddedFuelMedium.nXi] "CH4, C2H6, C3H8, n-C4H10, N2, CO2";
  FlueMedium.MassFraction X_air[FlueMedium.nXi] "N2, O2, H2O, CO2, SO2";
  FlueMedium.MassFraction X_exhaust[FlueMedium.nXi] "N2, O2, H2O, CO2, SO2";
  Real steelworksCombustibleMoles[6](each unit="mol/kg") "Per kg of steelworks gas";
  Real addedFuelCombustibleMoles[6](each unit="mol/kg") "Per kg of added fuel";
  SI.MolarFlowRate combustibleFlow[6];

equation
  Q_air = sink_air.Q_in;
  Q_fuel = sink_fuel.Q_in;
  Q_exhaust = -source_exhaust.Q_out;
  h_in_air = sink_air.h_in;
  h_in_fuel = sink_fuel.h_in;
  h_exhaust = source_exhaust.h_out;
  X_air = sink_air.Xi_in;
  X_fuel = sink_fuel.Xi_in;
  X_exhaust = source_exhaust.Xi_out;

  Q_added_fuel = sink_added_fuel.Q_in;
  h_in_added_fuel = if useAddedFuel then sink_added_fuel.h_in else 0;
  X_added_fuel = if useAddedFuel then sink_added_fuel.Xi_in else zeros(AddedFuelMedium.nXi);

  Q_exhaust = Q_air + Q_fuel + Q_added_fuel;
  source_exhaust.P_out = sink_air.P_in;

  steelworksCombustibleMoles = {X_fuel[2],X_fuel[4],X_fuel[5],0,0,0}
    ./ combustibleMolarMasses;
  addedFuelCombustibleMoles = {X_added_fuel[1],0,0,X_added_fuel[2],X_added_fuel[3],X_added_fuel[4]}
    ./ combustibleMolarMasses;
  combustibleFlow = Q_fuel*steelworksCombustibleMoles + Q_added_fuel*addedFuelCombustibleMoles;

  Q_exhaust*X_exhaust[1] = Q_air*X_air[1] + Q_fuel*X_fuel[1] + Q_added_fuel*X_added_fuel[5];
  Q_exhaust*X_exhaust[2] = Q_air*X_air[2] + Q_fuel*X_fuel[6]
    - (combustibleFlow*oxygenMoles)*Data.O2.MM;
  Q_exhaust*X_exhaust[3] = Q_air*X_air[3] + Q_fuel*X_fuel[7]
    + (combustibleFlow*waterMoles)*Data.H2O.MM;
  Q_exhaust*X_exhaust[4] = Q_air*X_air[4] + Q_fuel*X_fuel[3] + Q_added_fuel*X_added_fuel[6]
    + (combustibleFlow*carbonDioxideMoles)*Data.CO2.MM;
  Q_exhaust*X_exhaust[5] = Q_air*X_air[5];

  (steelworksHHV, steelworksCondensationHeat) =
    MetroscopeModelingLibrary.Utilities.Functions.combustionHeatingValues(
      {X_fuel[2],0,0,0}, hydrogen=X_fuel[4], carbonMonoxide=X_fuel[5]);
  (addedFuelHHV, addedFuelCondensationHeat) =
    MetroscopeModelingLibrary.Utilities.Functions.combustionHeatingValues(X_added_fuel[1:4]);

  // Energy balance in the enthalpy convention of the media.
  Wth = eta*(Q_fuel*LHV + Q_added_fuel*LHV_added_fuel);
  Q_exhaust*h_exhaust = Q_air*h_in_air + Q_fuel*h_in_fuel
    + Q_added_fuel*h_in_added_fuel + Wth;

  assert(Q_exhaust > 0, "SteelworkGasesBurner requires positive exhaust flow.");
  assert(X_exhaust[2] >= -1e-8, "SteelworkGasesBurner requires enough oxygen for complete combustion.");

  connect(inlet, sink_air.C_in)
    annotation (Line(points={{-100,0},{-27,0}}, color={95,95,95}));
  connect(inlet1, sink_fuel.C_in)
    annotation (Line(points={{-40,-100},{-40,-27}},
                                                color={213,170,255}));
  connect(fuel_in, sink_added_fuel.C_in)
    annotation (Line(points={{50,-100},{50,-51}}, color={213,213,0}));
  connect(idle_fuel.C_out, sink_added_fuel.C_in)
    annotation (Line(points={{79,-71},{50,-71},{50,-51}}, color={213,213,0}));
  connect(source_exhaust.C_out, pressure_loss.C_in)
    annotation (Line(points={{27,0},{46,0}}, color={95,95,95}));
  connect(pressure_loss.C_out, outlet)
    annotation (Line(points={{66,0},{100,0}}, color={95,95,95}));
  connect(pressure_loss.Kfr, Kfr)
    annotation (Line(points={{56,4},{56,100},{40,100}}, color={0,0,127}));

  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}},
      initialScale=0.2)),
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}},
      initialScale=0.2), graphics={
        Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,0},
          fillColor={244,154,189},fillPattern=FillPattern.Solid),
        Polygon(
          points={{-32,62},{-18.916,58.8281},{34,46},{-10,30},{46,18},{-6,0},
                  {36,-16},{-10,-32},{42,-44},{-20.4375,-57.5},{-32,-60},
                  {-32,-46},{-32,45.5586},{-32,62}},
          lineColor={158,0,0}, fillColor={255,0,0}, fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier)}),
    Documentation(info="<html>
<p>Steady-state burner using the air and exhaust topology of <code>CombustionChamber</code>. All combustible species are fully oxidized to carbon dioxide and water vapor. Set <code>useAddedFuel=true</code> to connect an additional <code>FuelMedium</code> stream to <code>fuel_in</code>. Its methane, ethane, propane and n-butane contribute to the oxygen demand and heating power. The air and steelworks gas supply the oxygen. Nitrogen and inlet sulfur dioxide pass through, with no NOx formation. The outlet medium represents complete combustion with sufficient oxygen.</p>
<p>The air inlet sets the chamber pressure upstream of the exhaust pressure loss. The upstream fuel circuits supply their respective pressures and flows independently. The parameter connector <code>eta</code> controls the net heat transferred to the products. The lower heating value at 298.15 K is the higher heating value less the condensation heat of the water formed, using the same convention as <code>CombustionChamber</code>.</p>
</html>"));
end SteelworkGasesBurner;
