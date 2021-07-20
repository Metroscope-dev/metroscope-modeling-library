within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model OpenSteamGenerator
  replaceable package water =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
public
  Modelica.Units.SI.Power ThermalPower(start=3000e6)
    "Total energy transfered from primary";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_WaterSteam "Singular pressure loss";
  Modelica.Units.SI.SpecificEnthalpy hvsat(start=2.7e6)
    "saturated steam at outlet specific enthalpy";
  Modelica.Units.SI.SpecificEnthalpy hlsat(start=1.2e6)
    "saturated liquid water specific enthalpy at purge";
  Real VaporFraction(start=0.99) "vapor fraction in steam at outlet";
  Real Q_residu(start=0) "GV mass balance residual";
public
  BoundaryConditions.Source steamSource annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,48})));
  BoundaryConditions.Sink waterSupplySink
    annotation (Placement(transformation(extent={{22,0},{-18,40}})));
  Common.Connectors.FluidInlet C_water_in
    annotation (Placement(transformation(extent={{36,10},{56,30}})));
  Common.Connectors.FluidOutlet C_steam_out
    annotation (Placement(transformation(extent={{-10,90},{10,112}})));
  Common.Connectors.FluidOutlet C_drain_out
    annotation (Placement(transformation(extent={{-8,-110},{12,-88}})));
  BoundaryConditions.Source drainSource annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={0,-42})));
equation
  hlsat=water.bubbleEnthalpy(water.setSat_p(drainSource.P_out));
  hvsat=water.dewEnthalpy(water.setSat_p(steamSource.P_out));
  ThermalPower=-steamSource.Q_out*steamSource.h_out -waterSupplySink.Q_in *
    waterSupplySink.h_in - drainSource.Q_out*drainSource.h_out;
  drainSource.h_out = hlsat;
  steamSource.h_out = VaporFraction*hvsat + (1 - VaporFraction)*hlsat;
  deltaP_WaterSteam=steamSource.P_out -waterSupplySink.P_in;
  waterSupplySink.h_vol = 500e3;
  Q_residu=waterSupplySink.Q_in +steamSource.Q_out  + drainSource.Q_out;
  connect(waterSupplySink.C_in, C_water_in)
    annotation (Line(points={{22,20},{46,20}}, color={63,81,181}));
  connect(steamSource.C_out, C_steam_out) annotation (Line(points={{1.11022e-15,
          68},{1.11022e-15,74.5},{0,74.5},{0,101}}, color={63,81,181}));
  connect(drainSource.C_out, C_drain_out) annotation (Line(points={{0,-62},{0,-80},
          {0,-99},{2,-99}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(
          points={{0,112},{-70,70},{-40,30},{-38,0},{-42,-98},{42,-98},{38,0},{
              40,32},{70,70},{0,112}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=
           false)));
end OpenSteamGenerator;
