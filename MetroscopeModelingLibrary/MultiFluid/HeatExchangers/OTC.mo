within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model OTC
  import MetroscopeModelingLibrary.Utilities.Units;

  parameter Units.Area S = 1000;
  FlueGases.Connectors.Inlet C_hot_in                                        annotation (Placement(transformation(
          extent={{-252,2},{-232,22}}), iconTransformation(extent={{-90,-48},{-70,
            -28}})));
  FlueGases.Connectors.Outlet C_hot_out                                                                          annotation (Placement(transformation(
          extent={{50,2},{70,22}}),   iconTransformation(extent={{-90,150},{-70,
            170}})));
  WaterSteam.Connectors.Inlet C_cold_in                                          annotation (Placement(transformation(
          extent={{30,82},{50,102}}),  iconTransformation(extent={{-50,110},{
            -30,130}})));
  WaterSteam.Connectors.Outlet C_cold_out                                                                            annotation (Placement(transformation(
          extent={{-150,-78},{-130,-58}}),
                                       iconTransformation(extent={{-50,-8},{-30,
            12}})));
  Evaporator evaporator(S_parameter=false)
    annotation (Placement(transformation(extent={{-70,-38},{30,152}})));
  Superheater superheater(S_parameter=false)
    annotation (Placement(transformation(extent={{-190,62},{-90,-38}})));
  FlueGases.Pipes.FrictionPipe Kfr_hot_pipe annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-90,12})));
  Utilities.Interfaces.GenericReal Kfr_cold annotation (Placement(
        transformation(extent={{-188,-32},{-180,-24}}),
                                                    iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,80})));
  Utilities.Interfaces.GenericReal Kth annotation (Placement(transformation(
          extent={{-188,48},{-180,56}}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-130,60})));
  Utilities.Interfaces.GenericReal Kfr_hot annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-90,32}),iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,40})));
equation
  evaporator.S_eq + superheater.S_eq = S;
  connect(C_hot_in, C_hot_in)
    annotation (Line(points={{-242,12},{-242,12}},
                                                 color={95,95,95},
      thickness=1));
  connect(C_cold_out, C_cold_out)
    annotation (Line(points={{-140,-68},{-140,-68}}, color={28,108,200},
      thickness=1));
  connect(superheater.C_hot_out, Kfr_hot_pipe.C_in)
    annotation (Line(points={{-120,12},{-100,12}},
                                             color={95,95,95},
      thickness=1));
  connect(Kfr_hot_pipe.Kfr, Kfr_hot)
    annotation (Line(points={{-90,16},{-90,32}},
                                              color={0,0,127},
      thickness=1));
  connect(C_hot_in, superheater.C_hot_in) annotation (Line(
      points={{-242,12},{-160,12}},
      color={95,95,95},
      thickness=1));
  connect(Kfr_hot_pipe.C_out, evaporator.C_hot_in) annotation (Line(
      points={{-80,12},{-60,12}},
      color={95,95,95},
      thickness=1));
  connect(evaporator.C_hot_out, C_hot_out) annotation (Line(
      points={{20,12},{60,12}},
      color={95,95,95},
      thickness=1));
  connect(Kfr_cold, superheater.Kfr_cold) annotation (Line(
      points={{-184,-28},{-162,-28}},
      color={0,0,127},
      thickness=1));
  connect(evaporator.C_cold_in, C_cold_in) annotation (Line(
      points={{15,92},{40,92}},
      color={28,108,200},
      thickness=1));
  connect(superheater.C_cold_out, C_cold_out) annotation (Line(
      points={{-140,-38},{-140,-68}},
      color={28,108,200},
      thickness=1));
  connect(superheater.C_cold_in, evaporator.C_cold_out) annotation (Line(
      points={{-140,62},{-140,132},{-55,132}},
      color={28,108,200},
      thickness=1));
  connect(Kth, superheater.Kth)
    annotation (Line(points={{-184,52},{-162,52}}, color={0,0,127}));
  connect(Kth, evaporator.Kth) annotation (Line(points={{-184,52},{-178,52},{
          -178,80},{-72,80},{-72,-28},{-62,-28}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -40},{0,160}},
        grid={2,2},
        initialScale=0.5),
                         graphics={Rectangle(
          extent={{-120,160},{-40,-40}},
          lineColor={28,108,200},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-160,-40},{0,160}},
        grid={2,2},
        initialScale=0.5)));
end OTC;
