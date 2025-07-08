within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  //Boundary Conditions
  input Utilities.Units.MassFlowRate Q_turbine(start=21) "kg/s";
  input Utilities.Units.SpecificEnthalpy h_turbine(start=2399e3) "J/kg";

  input Utilities.Units.MassFlowRate Q_cold(start=18000) "kg/s";
  input Real P_cold_source(start=1.002,nominal=1.002) "barA";
  input Real T_cold_source(start=10) "degC";
  input Utilities.Units.Fraction cold_source_relative_humidity(start=0.80) "1";


  // Sensors for calibration
  input Real P_cond(start=91) "mbarA";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet(h_out(start=2399000)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,70})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink(h_in(start=146568.66)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-62})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source(h_out(start=25400))
    annotation (Placement(transformation(extent={{-56,-10},{-36,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink(h_in(start=28028.451))    annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  MultiFluid.HeatExchangers.AirCooledCondenser                 airCooledCondenser(
    S=150000,
    P_incond=0,
    P_offset=0,
   cold_side_condensing(h_out(start=28243.033)))  annotation (Placement(transformation(extent={{-16,-16},{16,20}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cond_sensor
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,41})));
  Utilities.Interfaces.RealOutput ACC_Kth annotation (Placement(transformation(extent={{-32,34},{-24,42}}), iconTransformation(extent={{-116,26},{-96,46}})));
  Utilities.Interfaces.RealExpression ACC_Kfr_hot annotation (Placement(transformation(extent={{30,36},{50,56}})));
equation

  //Hot source
  turbine_outlet.h_out = h_turbine;
  turbine_outlet.Q_out = - Q_turbine;

  //Cold source
  cold_source.P_out = P_cold_source * 1e5;
  cold_source.T_out = T_cold_source + 273.15;
  cold_source.relative_humidity = cold_source_relative_humidity;
  cold_source.Q_out = - Q_cold;

  // Observable for calibration
  P_cond_sensor.P_mbar = P_cond;

  connect(airCooledCondenser.C_cold_out, cold_sink.C_in)
    annotation (Line(points={{16,0},{39,0}},   color={85,170,255}));
  connect(airCooledCondenser.C_cold_in, cold_source.C_out)
    annotation (Line(points={{-16,0},{-41,0}},   color={85,170,255}));
  connect(turbine_outlet.C_out, P_cond_sensor.C_in) annotation (Line(points={{0,65},{0,48},{1,48}},
                                     color={28,108,200}));
  connect(airCooledCondenser.C_hot_in, P_cond_sensor.C_out) annotation (Line(
        points={{0,20},{0,34},{1,34}},           color={28,108,200}));
  connect(condensate_sink.C_in, airCooledCondenser.C_hot_out) annotation (Line(points={{0,-57},{0,-35.5},{0,-35.5},{0,-16}},                     color={28,108,200}));
  connect(airCooledCondenser.Kth, ACC_Kth) annotation (Line(points={{-12.8,22},{-14,22},{-14,38},{-28,38}}, color={0,0,127}));
  connect(ACC_Kfr_hot.y, airCooledCondenser.Kfr_hot) annotation (Line(points={{40,41},{40,22},{12.8,22}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end AirCooledCondenser_reverse;
