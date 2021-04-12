within MetroscopeModelingLibrary.Multifluid.HeatExchangers;
model WaterFlueGasesEvaporator
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  replaceable package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  Modelica.SIunits.Power W(start=1e8);
  Modelica.SIunits.AbsolutePressure P(start=1e5);
  Real K_fg(start=1.e3) "Pressure loss coefficient";
  Modelica.SIunits.CoefficientOfHeatTransfer K(start = 100)
    "Global heat transfer coefficient (active if exchanger_type=3)";
  Modelica.SIunits.Area S(start = 10);
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_fg "Singular pressure loss";
  Modelica.SIunits.MassFlowRate Q_w(start=100) "Inlet Mass flow rate";
  Modelica.SIunits.MassFlowRate Q_fg(start=100) "Inlet Mass flow rate";
  Modelica.SIunits.Temperature Tsat(start = 150+273.15);
  replaceable Common.Partial.BasicTransportModel waterLiquidSide(redeclare
      package Medium = WaterSteamMedium) annotation (Placement(transformation(
        extent={{-25,-18},{25,18}},
        rotation=-90,
        origin={49,-52})));
  replaceable Common.Partial.BasicTransportModel flueGasesSide( redeclare
      package Medium =
        FlueGasesMedium)
    annotation (Placement(transformation(extent={{-28,4},{22,40}})));
  Common.Connectors.FluidInlet C_fg_in( redeclare package Medium =
        FlueGasesMedium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Common.Connectors.FluidOutlet C_fg_out(redeclare package Medium =
        FlueGasesMedium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Common.Connectors.FluidInlet C_w_in( redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-10,80},{10,100}}),
        iconTransformation(extent={{-10,80},{10,100}})));
  Common.Connectors.FluidOutlet C_w_liquid_out(redeclare package Medium =
        WaterSteamMedium) annotation (Placement(transformation(extent={{40,-100},
            {60,-80}}), iconTransformation(extent={{40,-100},{60,-80}})));
  replaceable Common.Partial.BasicTransportModel waterVaporSide(redeclare
      package Medium = WaterSteamMedium) annotation (Placement(transformation(
        extent={{-25,-18},{25,18}},
        rotation=-90,
        origin={-49,-50})));
  Common.Connectors.FluidOutlet C_w_vapor_out(redeclare package Medium =
        WaterSteamMedium) annotation (Placement(transformation(extent={{-60,-100},
            {-40,-80}}), iconTransformation(extent={{-60,-100},{-40,-80}})));
equation
  Q_w = waterLiquidSide.Q_in + waterVaporSide.Q_in;
  Q_fg = flueGasesSide.Q_in;
  //Mechanical Balance
  deltaP_fg = flueGasesSide.P_in - flueGasesSide.P_out;
  deltaP_fg = K_fg*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(flueGasesSide.Q_in, flueGasesSide.eps)/flueGasesSide.rhom;
  P = waterLiquidSide.P_out;
  P = waterVaporSide.P_out;
  P = waterLiquidSide.P_in;
  //Mass Balance
  flueGasesSide.Q_in + flueGasesSide.Q_out = 0;
  waterLiquidSide.Q_in + waterLiquidSide.Q_out = 0;
  waterVaporSide.Q_in + waterVaporSide.Q_out = 0;
  //Saturation
  waterLiquidSide.h_out=WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P));
  waterVaporSide.h_out=WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P));
  //Energy balance
  waterLiquidSide.Q_in*waterLiquidSide.h_in + waterLiquidSide.Q_out*waterLiquidSide.h_out + waterVaporSide.Q_in*waterVaporSide.h_in + waterVaporSide.Q_out*waterVaporSide.h_out = -W;
  flueGasesSide.Q_in*flueGasesSide.h_in + flueGasesSide.Q_out*flueGasesSide.h_out = W;
  waterLiquidSide.Q_in*waterLiquidSide.Xi_in =- waterLiquidSide.Q_out*waterLiquidSide.Xi_out;
  waterVaporSide.Q_in*waterVaporSide.Xi_in =- waterVaporSide.Q_out*waterVaporSide.Xi_out;
  flueGasesSide.Q_in*flueGasesSide.Xi_in =- flueGasesSide.Q_out*flueGasesSide.Xi_out;

  Tsat = waterVaporSide.T_out;
  0 = Tsat - flueGasesSide.T_out - (Tsat - flueGasesSide.T_in)*exp(K*S*((flueGasesSide.T_out - flueGasesSide.T_in)/W));
  connect(C_fg_in, flueGasesSide.C_in) annotation (Line(points={{-100,0},{-52,0},
          {-52,22},{-28,22}}, color={0,0,255}));
  connect(flueGasesSide.C_out, C_fg_out) annotation (Line(points={{22.5,22},{62,
          22},{62,0},{100,0}}, color={238,46,47}));
  connect(waterLiquidSide.C_out, C_w_liquid_out) annotation (Line(points={{49,
          -77.5},{48,-77.5},{48,-90},{50,-90}},
                                         color={238,46,47}));
  connect(waterVaporSide.C_out, C_w_vapor_out) annotation (Line(points={{-49,
          -75.5},{-49,-76.75},{-50,-76.75},{-50,-90}},
                                                color={238,46,47}));
  connect(waterLiquidSide.C_in, waterVaporSide.C_in) annotation (Line(points={{49,
          -27},{49,-10},{-49,-10},{-49,-25}}, color={0,0,255}));
  connect(C_w_in, waterVaporSide.C_in) annotation (Line(points={{0,90},{0,62},{
          -68,62},{-68,-4},{0,-4},{0,-10},{-49,-10},{-49,-25}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,80},{100,-80}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={255,255,0},
          fillPattern=FillPattern.Backward),
        Polygon(
          points={{-94,12},{-80,12},{-80,56},{80,56},{80,12},{92,12},{92,6},{74,
              6},{74,50},{-74,50},{-74,6},{-94,6},{-94,12}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-92,3},{92,3},{92,-3},{-92,-3},{-92,3}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-94,-12},{-80,-12},{-80,-56},{80,-56},{80,-12},{92,-12},{92,
              -6},{74,-6},{74,-50},{-74,-50},{-74,-6},{-94,-6},{-94,-12}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end WaterFlueGasesEvaporator;
