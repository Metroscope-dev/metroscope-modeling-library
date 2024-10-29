within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.Multiple_pass_HX;
model HX_3_pass
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Number of tubes per water flow direction
  parameter Integer N_tubes_row_dir_1 = 62 "Number of tubes of water flowing in in direction 1";
  parameter Integer N_tubes_row_dir_2 = 62 "Number of tubes of water flowing in in direction 2";
  parameter Integer N_tubes_row_dir_3 = 62 "Number of tubes of water flowing in in direction 2";

  // Indicators
    // Configuration
    Integer N_tubes_row_tot "Total number of tubes per row";
    Units.PositiveMassFlowRate Q_fg_dir_1 "Flue gas mass flow rate heating water in direction 1";
    Units.PositiveMassFlowRate Q_fg_dir_2 "Flue gas mass flow rate heating water in direction 2";
    Units.PositiveMassFlowRate Q_fg_dir_3 "Flue gas mass flow rate heating water in direction 3";
    Units.PositiveMassFlowRate Q_fg_tot "Total flue gas mass flow rate";
    // Temperatures
    Units.Temperature T_water_in "Water inlet temperature";
    Units.Temperature T_water_out "Water outlet temperature";
    Units.Temperature T_fg_in "Flue gas inlet temperature";
    Units.Temperature T_fg_out "Flue gas outlet temperature";

  One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_1(
    N_tubes_row=N_tubes_row_dir_1,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{-50,-11},{-30,11}})));

  One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_2(
    N_tubes_row=N_tubes_row_dir_3,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{-10,11},{10,-11}})));
  One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_3(
    N_tubes_row=N_tubes_row_dir_2,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{30,-11},{50,11}})));
  FlueGases.Pipes.PressureCut PC_1 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-28,30})));
  FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-70,-110},{-50,-90}}), iconTransformation(extent={{-70,-110},{-50,-90}})));
  WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{50,90},{70,110}})));
  FlueGases.BaseClasses.IsoPHFlowModel fg_inlet_properties annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-70,0})));
  FlueGases.BaseClasses.IsoPHFlowModel fg_outlet_properties
                                                           annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={70,0})));
  FlueGases.Pipes.PressureCut PC_2 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={20,30})));
equation

  // Indicators
    // Configuration
    N_tubes_row_tot = N_tubes_row_dir_1 + N_tubes_row_dir_2 + N_tubes_row_dir_3;
    Dir_1.fg_side.Q = Q_fg_dir_1;
    Dir_2.fg_side.Q = Q_fg_dir_2;
    Dir_3.fg_side.Q = Q_fg_dir_3;
    Q_fg_tot = Q_fg_dir_1 + Q_fg_dir_2 + Q_fg_dir_3;
    // Temperatures
    T_water_in = Dir_1.T_water_in;
    T_water_out = Dir_3.T_water_out;
    T_fg_in = fg_inlet_properties.T_in;
    T_fg_out = fg_outlet_properties.T_out;

  // Flow repartition is proportional to the number of tubes
  Q_fg_dir_1 = Q_fg_tot*N_tubes_row_dir_1/N_tubes_row_tot;
  Q_fg_dir_2 = Q_fg_tot*N_tubes_row_dir_2/N_tubes_row_tot;

  connect(fg_inlet, fg_inlet) annotation (Line(points={{-100,0},{-100,0}}, color={95,95,95}));
  connect(fg_inlet, fg_inlet_properties.C_in) annotation (Line(points={{-100,0},{-80,0}},           color={95,95,95}));
  connect(fg_inlet_properties.C_out, Dir_1.fg_inlet) annotation (Line(points={{-60,0},{-44,0}}, color={95,95,95}));
  connect(fg_outlet_properties.C_out, fg_outlet) annotation (Line(points={{80,0},{100,0}}, color={95,95,95}));
  connect(Dir_3.fg_outlet, fg_outlet_properties.C_in) annotation (Line(points={{44,0},{60,0}}, color={95,95,95}));
  connect(PC_1.C_in, Dir_1.fg_outlet) annotation (Line(points={{-28,20},{-28,0},{-36,0}}, color={95,95,95}));
  connect(PC_1.C_out, fg_outlet_properties.C_in) annotation (Line(points={{-28,40},{-28,60},{52,60},{52,0},{60,0}}, color={95,95,95}));
  connect(PC_2.C_out, fg_outlet_properties.C_in) annotation (Line(points={{20,40},{20,60},{52,60},{52,0},{60,0}}, color={95,95,95}));
  connect(PC_2.C_in,Dir_2. fg_outlet) annotation (Line(points={{20,20},{22,20},{22,0},{4,0}}, color={95,95,95}));
  connect(Dir_2.fg_inlet, Dir_1.fg_inlet) annotation (Line(points={{-4,0},{-20,0},{-20,-40},{-52,-40},{-52,0},{-44,0}}, color={95,95,95}));
  connect(Dir_3.fg_inlet, Dir_1.fg_inlet) annotation (Line(points={{36,0},{28,0},{28,-40},{-52,-40},{-52,0},{-44,0}}, color={95,95,95}));
  connect(Dir_1.water_inlet, inlet) annotation (Line(points={{-40,-11},{-40,-60},{-60,-60},{-60,-100}}, color={28,108,200}));
  connect(Dir_1.water_outlet, Dir_2.water_inlet) annotation (Line(points={{-40,11},{-40,80},{0,80},{0,11}}, color={28,108,200}));
  connect(Dir_2.water_outlet, Dir_3.water_inlet) annotation (Line(points={{0,-11},{0,-60},{40,-60},{40,-11}}, color={28,108,200}));
  connect(Dir_3.water_outlet, outlet) annotation (Line(points={{40,11},{40,80},{60,80},{60,100}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Ellipse(extent={{-72,70},{-72,70}}, lineColor={28,108,200}),
        Rectangle(
          extent={{-80,86},{80,80}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-80,-80},{80,-86}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{30,86},{30,80}},
          color={0,0,0},
          thickness=1),
        Line(points={{60,0},{60,80}},   color={28,108,200}),
        Line(
          points={{-30,-80},{-30,-86}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{-60,-80},{-60,-62},{-60,80},{0,80},{0,-80},{60,-80},{60,80}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Polygon(
          points={{-60,8},{-66,-12},{-54,-12},{-60,8}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,-14},{-6,6},{6,6},{0,-14}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{60,8},{54,-12},{66,-12},{60,8}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-92,16},{-72,-4}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{-42,16},{-22,-4}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{20,16},{40,-4}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{70,18},{90,-2}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled})}),                    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=500,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end HX_3_pass;
