within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model OTC
  import MetroscopeModelingLibrary.Utilities.Units;

  parameter Units.Area S = 1000;
  FlueGases.Connectors.Inlet C_hot_in                                        annotation (Placement(transformation(
          extent={{-232,-10},{-212,10}}),
                                        iconTransformation(extent={{-90,-48},{-70,
            -28}})));
  FlueGases.Connectors.Outlet C_hot_out                                                                          annotation (Placement(transformation(
          extent={{70,-10},{90,10}}), iconTransformation(extent={{-90,150},{-70,
            170}})));
  WaterSteam.Connectors.Inlet C_cold_in                                          annotation (Placement(transformation(
          extent={{50,70},{70,90}}),   iconTransformation(extent={{-50,110},{
            -30,130}})));
  WaterSteam.Connectors.Outlet C_cold_out                                                                            annotation (Placement(transformation(
          extent={{-130,-90},{-110,-70}}),
                                       iconTransformation(extent={{-50,-8},{-30,
            12}})));
  Evaporator evaporator(S_parameter=false)
    annotation (Placement(transformation(extent={{-50,-50},{50,140}})));
  Superheater superheater(S_parameter=false)
    annotation (Placement(transformation(extent={{-170,50},{-70,-50}})));
  FlueGases.Pipes.FrictionPipe Kfr_hot_pipe annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-70,0})));
  Utilities.Interfaces.GenericReal Kth_evaporating annotation (Placement(
        transformation(extent={{-72,-44},{-64,-36}}),   iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-130,80})));
  Utilities.Interfaces.GenericReal Kfr_cold annotation (Placement(
        transformation(extent={{-168,-44},{-160,-36}}),
                                                    iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,60})));
  Utilities.Interfaces.GenericReal Kth_superheating annotation (Placement(
        transformation(extent={{-168,36},{-160,44}}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-130,40})));
  Utilities.Interfaces.GenericReal Kfr_hot annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-70,20}),iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-110,-50})));
equation
  evaporator.S + superheater.S = S;
  connect(C_hot_in, C_hot_in)
    annotation (Line(points={{-222,0},{-222,0}}, color={95,95,95},
      thickness=1));
  connect(C_cold_out, C_cold_out)
    annotation (Line(points={{-120,-80},{-120,-80}}, color={28,108,200},
      thickness=1));
  connect(superheater.C_hot_out, Kfr_hot_pipe.C_in)
    annotation (Line(points={{-100,0},{-80,0}},
                                             color={95,95,95},
      thickness=1));
  connect(Kth_evaporating, evaporator.Kth)
    annotation (Line(points={{-68,-40},{-42,-40}},   color={0,0,127},
      thickness=1));
  connect(Kfr_hot_pipe.Kfr, Kfr_hot)
    annotation (Line(points={{-70,4},{-70,20}},
                                              color={0,0,127},
      thickness=1));
  connect(C_hot_in, superheater.C_hot_in) annotation (Line(
      points={{-222,0},{-140,0}},
      color={95,95,95},
      thickness=1));
  connect(Kfr_hot_pipe.C_out, evaporator.C_hot_in) annotation (Line(
      points={{-60,0},{-40,0}},
      color={95,95,95},
      thickness=1));
  connect(evaporator.C_hot_out, C_hot_out) annotation (Line(
      points={{40,0},{80,0}},
      color={95,95,95},
      thickness=1));
  connect(Kfr_cold, superheater.Kfr_cold) annotation (Line(
      points={{-164,-40},{-142,-40}},
      color={0,0,127},
      thickness=1));
  connect(Kth_superheating, superheater.Kth) annotation (Line(
      points={{-164,40},{-142,40}},
      color={0,0,127},
      thickness=1));
  connect(evaporator.C_cold_in, C_cold_in) annotation (Line(
      points={{35,80},{60,80}},
      color={28,108,200},
      thickness=1));
  connect(superheater.C_cold_out, C_cold_out) annotation (Line(
      points={{-120,-50},{-120,-80}},
      color={28,108,200},
      thickness=1));
  connect(superheater.C_cold_in, evaporator.C_cold_out) annotation (Line(
      points={{-120,50},{-120,120},{-35,120}},
      color={28,108,200},
      thickness=1));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-40},
            {0,160}}),   graphics={Rectangle(
          extent={{-120,160},{-40,-40}},
          lineColor={28,108,200},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-160,-40},{0,160}})));
end OTC;
