within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.Simplified;
model Simplified_model "Added more info on fins and used ESCOA correlation"
 import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import CorrelationConstants =
         MetroscopeModelingLibrary.DynamicComponents.Correlations;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    parameter Modelica.Units.SI.ThermalConductance UA = 264900 "W/K"; // = 264900

    // Real UA "Overall heat transfer coefficient and exchange surface product";
    // Temperatures
    Units.Temperature T_water_in "Water inlet temperature";
    Units.Temperature T_water_out "Water outlet temperature";
    Units.Temperature T_fg_in "Flue gas inlet temperature";
    Units.Temperature T_fg_out "Flue gas outlet temperature";
    // Pressures
    Units.Pressure P_water(start=P_water_0) "Water Pressure";
    Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
    // Mass flow rates
    Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
    Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
    // Heat exchanged
    Units.Power W_water;
    Units.Power W_fg;

    Units.Temperature DT_LMTD;

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

  WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                          iconTransformation(extent={{-10,-110},{10,-90}})));
  WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
  FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
  FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
  WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={0,20})));
  FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
equation

  // ------ Boundaries ------
    // Enthalpy
    T_water_in = water_side.T_in;
    T_water_out = water_side.T_out;
    T_fg_in = fg_side.T_in;
    T_fg_out = fg_side.T_out;
    P_water = water_side.P_in;
    P_fg = fg_side.P_in;
    // Mass flow rate
    Q_water = water_side.Q;
    Q_fg = fg_side.Q;
    // Mass Fraction
    water_side.W = W_water;
    fg_side.W = W_fg;

  // Energy Balance
    W_water = -W_fg;
    DT_LMTD = ((T_fg_out - T_water_in) - (T_fg_in - T_water_out))/log((T_fg_out - T_water_in)/(T_fg_in - T_water_out));
    W_water = UA*DT_LMTD;

  connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                   color={28,108,200}));
  connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                       color={28,108,200}));
  connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                 color={95,95,95}));
  connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                 color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,10},{100,-10}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          origin={-30,0},
          rotation=90),
        Rectangle(
          extent={{-100,5},{100,-5}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          origin={-15,0},
          rotation=90),
        Rectangle(
          extent={{-100,10},{100,-10}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=90),
        Rectangle(
          extent={{-100,5},{100,-5}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          origin={15,0},
          rotation=90),
        Rectangle(
          extent={{-100,10},{100,-10}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          origin={30,0},
          rotation=90),
        Line(points={{-40,-80},{40,-80}},color={0,0,0}),
        Line(points={{-40,-60},{40,-60}},color={0,0,0}),
        Line(points={{-40,-20},{40,-20}},color={0,0,0}),
        Line(points={{-40,-40},{40,-40}},color={0,0,0}),
        Line(points={{-40,80},{40,80}},  color={0,0,0}),
        Line(points={{-40,60},{40,60}},  color={0,0,0}),
        Line(points={{-40,40},{40,40}},  color={0,0,0}),
        Line(points={{-40,20},{40,20}},  color={0,0,0}),
        Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
end Simplified_model;
