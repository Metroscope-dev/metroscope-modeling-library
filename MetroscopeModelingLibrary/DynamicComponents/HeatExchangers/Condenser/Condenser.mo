within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.Condenser;
model Condenser "Added more info on fins and used ESCOA correlation"
 import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import CorrelationConstants =
         MetroscopeModelingLibrary.DynamicComponents.Correlations;

  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    Modelica.Units.SI.ThermalConductance UA "W/K"; // = 264900
    Modelica.Units.SI.ThermalConductance UA_hot "W/K"; // = 264900
    Modelica.Units.SI.ThermalConductance UA_cold "W/K"; // = 264900

    parameter Real r_UA = 3;

    parameter Boolean steady_state = false;

    // Wall
    parameter Units.Mass M_wall = 25379 "Tubes total mass";
    parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

    // Temperatures
    Units.Temperature T_cold_in "Cooling inlet temperature";
    Units.Temperature T_cold_out "Cooling water outlet temperature";
    Units.Temperature T_cold_avg "Cooling water average temperature";
    Units.Temperature T_hot_in "Cooling inlet temperature";
    Units.Temperature T_hot_out "Cooling water outlet temperature";
    Units.Temperature T_hot_sat "Saturation temperature";
    Units.Temperature T_wall "Wall temperature";
    // Pressures
    Units.Pressure P_cold_in(start=P_water_0) "Cooling water inlet pressure";
    Units.Pressure P_cold_out(start=P_water_0) "Cooling water outlet pressure";
    Units.Pressure P_sat(start=P_fg_0) "Saturation pressure";
    // Mass flow rates
    Units.PositiveMassFlowRate Q_cold(start=Q_water_0) "Cold water mass flow rate";
    Units.VolumeFlowRate Qv_cold "Cold water volumetric flow rate";
    Units.PositiveMassFlowRate Q_hot(start=Q_fg_0) "Hot water mass flow rate";
    // Heat exchanged
    Units.Power W_cold;
    Units.Power W_hot;
    // Saturation state
    Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat "Saturation state for properties calculation";

    // ------ Initialization ------
    parameter Units.Temperature T_wall_0 = 450;
    parameter Units.Pressure P_water_0 = 70e5;
    parameter Units.Pressure P_fg_0 = 1e5;
    parameter Units.PositiveMassFlowRate Q_water_0 = 85;
    parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
    parameter Units.Temperature T_water_out_0 = 500;
    parameter Units.Temperature T_fg_out_0 = 560;
    parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
    parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
    parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

  WaterSteam.Connectors.Inlet C_cold_in annotation (Placement(transformation(extent={{90,-30},{110,-10}}), iconTransformation(extent={{90,-30},{110,-10}})));
  WaterSteam.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{90,10},{110,30}}), iconTransformation(extent={{90,10},{110,30}})));
  WaterSteam.Connectors.Inlet C_hot_in annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
  WaterSteam.Connectors.Outlet C_hot_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
  WaterSteam.Pipes.HeatLoss hot_side annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,40})));
  WaterSteam.Pipes.HeatLoss cold_side annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-30,20})));
equation

initial equation
  if not steady_state then
  der(T_wall) = 0;
  end if;

equation

  r_UA = UA_cold/UA_hot;
  1/UA = 1/UA_cold + 1/UA_hot;

  // ------ Boundaries ------
    // Enthalpy
    T_cold_in = cold_side.T_in;
    T_cold_out = cold_side.T_out;
    T_hot_in = hot_side.T_in;
    T_hot_out = hot_side.T_out;
    P_cold_in = cold_side.P_in;
    P_cold_out = cold_side.P_out;
    P_sat = hot_side.P_in;
    // Mass flow rate
    Q_cold = cold_side.Q;
    Qv_cold = cold_side.Qv_in;
    Q_hot = hot_side.Q;
    // Mass Fraction
    cold_side.W = W_cold;
    hot_side.W = W_hot;

  // Saturation
  // Set saturation state
    sat.psat = P_sat;
    sat.Tsat = WaterSteamMedium.saturationTemperature(P_sat);
    T_hot_sat = sat.Tsat;
    hot_side.h_out = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_sat));

  // Average cold water temperature
    T_cold_avg = (T_cold_in + T_cold_out)/2;

  // Energy Balance
    if steady_state then
      W_cold + W_hot = 0;
    else
      W_cold + W_hot + M_wall*Cp_wall*der(T_wall) = 0;
    end if;

    W_cold = UA_cold*(T_wall - T_cold_avg);
    W_hot = - UA_hot*(T_hot_sat - T_wall);

  connect(hot_side.C_in, C_hot_in) annotation (Line(points={{-20,40},{0,40},{0,100}}, color={28,108,200}));
  connect(hot_side.C_out, C_hot_out) annotation (Line(points={{-40,40},{-80,40},{-80,-80},{0,-80},{0,-100}}, color={28,108,200}));
  connect(cold_side.C_in, C_cold_in) annotation (Line(points={{-40,20},{-60,20},{-60,-20},{100,-20}}, color={28,108,200}));
  connect(cold_side.C_out, C_cold_out) annotation (Line(points={{-20,20},{100,20}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{-40,100},{-100,60},{-100,-46},{100,-46},{100,60},{40,100},{-40,100}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-80,26},{-28,-26}},
          lineColor={0,0,255},
          lineThickness=0.5),
        Ellipse(
          extent={{-74,20},{-34,-20}},
          lineColor={0,0,255},
          lineThickness=0.5),
        Ellipse(
          extent={{-68,14},{-40,-14}},
          lineColor={0,0,255},
          lineThickness=0.5),
        Rectangle(
          extent={{-24,-26},{-54,28}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Line(
          points={{-54,26},{90,26}},
          color={0,0,255},
          thickness=0.5),
        Line(
          points={{-54,20},{90,20}},
          color={0,0,255},
          thickness=0.5),
        Line(
          points={{-54,14},{90,14}},
          color={0,0,255},
          thickness=0.5),
        Line(
          points={{-54,-14},{88,-14}},
          color={0,0,255},
          thickness=0.5),
        Line(
          points={{-54,-20},{88,-20}},
          color={0,0,255},
          thickness=0.5),
        Line(
          points={{-54,-26},{88,-26}},
          color={0,0,255},
          thickness=0.5),
        Polygon(
          points={{42,28},{46,26},{42,24},{42,28}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{42,22},{46,20},{42,18},{42,22}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{42,16},{46,14},{42,12},{42,16}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{46,-12},{42,-14},{46,-16},{46,-12}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{46,-18},{42,-20},{46,-22},{46,-18}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{46,-24},{42,-26},{46,-28},{46,-24}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-46},{100,-100}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-50,8},{-46,4}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-36,-36},{-32,-40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,-38},{-54,-42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-34,4},{-30,0}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{26,8},{22,4}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,-34},{-16,-38}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{42,10},{38,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{30,-40},{34,-44}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-16,10},{-12,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{2,6},{6,2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-78,-28},{-74,-32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,-34},{20,-38}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{76,6},{80,2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{54,-6},{50,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{68,-30},{72,-34}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-16,-2},{-12,-6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-46,-6},{-42,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{2,-30},{6,-34}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{52,-36},{56,-40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end Condenser;
