within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.Multiple_pass_HX;
model HX_4_pass
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Number of tubes per water flow direction
  parameter Integer N_tubes_row_dir_1 = 40 "Number of tubes of water flowing in in direction 1";
  parameter Integer N_tubes_row_dir_2 = 40 "Number of tubes of water flowing in in direction 2";
  parameter Integer N_tubes_row_dir_3 = 40 "Number of tubes of water flowing in in direction 3";
  parameter Integer N_tubes_row_dir_4 = 40 "Number of tubes of water flowing in in direction 4";

  // Indicators
    // Configuration
    Integer N_tubes_row_tot "Total number of tubes per row";
    Units.PositiveMassFlowRate Q_fg_dir_1 "Flue gas mass flow rate heating water in direction 1";
    Units.PositiveMassFlowRate Q_fg_dir_2 "Flue gas mass flow rate heating water in direction 2";
    Units.PositiveMassFlowRate Q_fg_dir_3 "Flue gas mass flow rate heating water in direction 3";
    Units.PositiveMassFlowRate Q_fg_dir_4 "Flue gas mass flow rate heating water in direction 4";
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
    T_wall_0=745.15) annotation (Placement(transformation(extent={{-70,11},{-50,-11}})));

  One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_2(
    N_tubes_row=N_tubes_row_dir_2,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{-30,-11},{-10,11}})));
  One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_3(
    N_tubes_row=N_tubes_row_dir_3,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{10,11},{30,-11}})));
  One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_4(
    N_tubes_row=N_tubes_row_dir_4,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{50,-11},{70,11}})));
  FlueGases.Pipes.PressureCut PC_1 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-46,20})));
  FlueGases.Pipes.PressureCut PC_2 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,20})));
  FlueGases.Pipes.PressureCut PC_3 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,20})));
  FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-70,90},{-50,110}}),   iconTransformation(extent={{-70,90},{-50,110}})));
  WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{50,90},{70,110}})));
  FlueGases.BaseClasses.IsoPHFlowModel fg_inlet_properties annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,-20})));
  FlueGases.BaseClasses.IsoPHFlowModel fg_outlet_properties
                                                           annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={80,-22})));
equation

  // Indicators
    // Configuration
    N_tubes_row_tot = Dir_1.N_tubes_row + Dir_2.N_tubes_row + Dir_3.N_tubes_row + Dir_4.N_tubes_row;
    Dir_1.fg_side.Q =  Q_fg_dir_1;
    Dir_2.fg_side.Q =  Q_fg_dir_2;
    Dir_3.fg_side.Q =  Q_fg_dir_2;
    Dir_4.fg_side.Q =  Q_fg_dir_3;
    Q_fg_tot = Q_fg_dir_1 + Q_fg_dir_2 + Q_fg_dir_3 + Q_fg_dir_4;
    // Temperatures
    T_water_in = Dir_1.T_water_in;
    T_water_out = Dir_4.T_water_out;
    T_fg_in = fg_inlet_properties.T_in;
    T_fg_out = fg_outlet_properties.T_out;

  // Flow repartition is proportional to the number of tubes
  Q_fg_dir_1 = Q_fg_tot*Dir_1.N_tubes_row/N_tubes_row_tot;
  Q_fg_dir_2 = Q_fg_tot*Dir_2.N_tubes_row/N_tubes_row_tot;
  Q_fg_dir_3 = Q_fg_tot*Dir_3.N_tubes_row/N_tubes_row_tot;

  connect(Dir_1.fg_outlet, PC_1.C_in) annotation (Line(points={{-56,0},{-46,0},{-46,10}}, color={95,95,95}));
  connect(Dir_2.fg_outlet, PC_2.C_in) annotation (Line(points={{-16,0},{0,0},{0,10}}, color={95,95,95}));
  connect(Dir_3.fg_outlet, PC_3.C_in) annotation (Line(points={{24,0},{40,0},{40,10}}, color={95,95,95}));
  connect(fg_inlet, fg_inlet) annotation (Line(points={{-100,0},{-100,0}}, color={95,95,95}));
  connect(fg_inlet, fg_inlet_properties.C_in) annotation (Line(points={{-100,0},{-80,0},{-80,-10}}, color={95,95,95}));
  connect(fg_inlet_properties.C_out, Dir_4.fg_inlet) annotation (Line(points={{-80,-30},{-80,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
  connect(Dir_1.fg_inlet, Dir_4.fg_inlet) annotation (Line(points={{-64,0},{-70,0},{-70,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
  connect(Dir_2.fg_inlet, Dir_4.fg_inlet) annotation (Line(points={{-24,0},{-30,0},{-30,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
  connect(Dir_3.fg_inlet, Dir_4.fg_inlet) annotation (Line(points={{16,0},{10,0},{10,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
  connect(Dir_4.water_outlet, outlet) annotation (Line(points={{60,11},{60,100},{60,100}}, color={28,108,200}));
  connect(Dir_1.water_inlet, inlet) annotation (Line(points={{-60,11},{-60,100},{-60,100}}, color={28,108,200}));
  connect(Dir_1.water_outlet, Dir_2.water_inlet) annotation (Line(points={{-60,-11},{-60,-60},{-20,-60},{-20,-11}}, color={28,108,200}));
  connect(Dir_2.water_outlet, Dir_3.water_inlet) annotation (Line(points={{-20,11},{-20,60},{20,60},{20,11}}, color={28,108,200}));
  connect(Dir_3.water_outlet, Dir_4.water_inlet) annotation (Line(points={{20,-11},{20,-60},{60,-60},{60,-11}}, color={28,108,200}));
  connect(Dir_4.fg_outlet, fg_outlet_properties.C_in) annotation (Line(points={{64,0},{72,0},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
  connect(PC_1.C_out, fg_outlet_properties.C_in) annotation (Line(points={{-46,30},{-46,40},{72,40},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
  connect(PC_2.C_out, fg_outlet_properties.C_in) annotation (Line(points={{0,30},{0,40},{72,40},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
  connect(PC_3.C_out, fg_outlet_properties.C_in) annotation (Line(points={{40,30},{40,40},{72,40},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
  connect(fg_outlet_properties.C_out, fg_outlet) annotation (Line(points={{80,-12},{80,0},{100,0}}, color={95,95,95}));
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
          points={{-40,86},{-40,80}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{40,86},{40,80}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{0,-80},{0,-86}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{-60,80},{-60,-82},{-20,-80},{-20,80},{20,80},{20,-80},{60,-80},{60,80}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Polygon(
          points={{-60,-10},{-66,10},{-54,10},{-60,-10}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-20,12},{-26,-8},{-14,-8},{-20,12}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{20,-10},{14,10},{26,10},{20,-10}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{60,10},{54,-10},{66,-10},{60,10}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{30,20},{50,0}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{-10,20},{10,0}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{-50,20},{-30,0}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{-92,20},{-72,0}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{70,20},{90,0}},
          color={0,0,0},
          thickness=1,
          arrow={Arrow.None,Arrow.Filled})}),                    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=500,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end HX_4_pass;
