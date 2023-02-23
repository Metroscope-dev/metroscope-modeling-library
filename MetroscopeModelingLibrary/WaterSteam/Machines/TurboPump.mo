within MetroscopeModelingLibrary.WaterSteam.Machines;
model TurboPump "turbine-driven pump"
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Pump variables
  Real VRotn(start=1400, min=0, nominal=2000) "Nominal rotational speed";
  Inputs.InputReal a1(start=0) "x^2 coef. of the pump characteristics hn = f(vol_flow) (s2/m5)";
  Inputs.InputReal a2(start=0) "x coef. of the pump characteristics hn = f(vol_flow) (s/m2)";
  Inputs.InputHeight a3(start=284.54654) "Constant coef. of the pump characteristics hn = f(vol_flow) (m)";
  Inputs.InputReal b1(start=0) "x^2 coef. of the pump efficiency characteristics rh = f(vol_flow) (s2/m6)";
  Inputs.InputReal b2(start=0) "x coef. of the pump efficiency characteristics rh = f(vol_flow) (s/m3)";
  Inputs.InputYield b3(start=0.845) "Constant coef. of the pump efficiency characteristics rh = f(vol_flow) (s.u.)";

  Units.Yield rh(start=1) "Hydraulic efficiency";
  Inputs.InputYield rm(start=1) "Mechanical efficiency";
  Inputs.InputYield rhmin(start=0.2) "Minimal hydraulic efficiency";
  Units.Height hn(start=227.92888) "Pump head";

  // Turbine variables
  Inputs.InputCst Cst "Stodola's ellipse coefficient";
  Inputs.InputYield eta_is(start=0.588) "Nominal isentropic efficiency";

  // Initialization parameters
  parameter Units.PositiveMassFlowRate Q_turbine_0 = 10;
  parameter Units.Pressure P_turbine_in_0 = 15e5;
  parameter Units.Pressure P_turbine_out_0 = 0.05e5;
  parameter Units.SpecificEnthalpy h_turbine_in_0 = 15e5;
  parameter Units.SpecificEnthalpy h_turbine_out_0 = 0.05e5;
  parameter Units.PositiveMassFlowRate Q_pump_0 = 1500;
  parameter Units.Pressure P_pump_in_0 = 30e5;
  parameter Units.Pressure P_pump_out_0 = 60e5;
  parameter Units.Temperature T_pump_in_0 = 200;
  parameter Units.Temperature T_pump_out_0 = 205;

  StodolaTurbine turbine annotation (Placement(transformation(extent={{-10,50},{10,30}})));
  Pump pump annotation (Placement(transformation(extent={{10,-130},{-10,-110}})));
  Connectors.Inlet C_pump_in annotation (Placement(transformation(extent={{90,-150},{110,-130}}), iconTransformation(extent={{90,-150},{110,-130}})));
  Connectors.Outlet C_pump_out annotation (Placement(transformation(extent={{-110,-150},{-90,-130}}), iconTransformation(extent={{-110,-150},{-90,-130}})));
  Connectors.Inlet C_turbine_in annotation (Placement(transformation(extent={{-70,50},{-50,70}}), iconTransformation(extent={{-70,50},{-50,70}})));
  Connectors.Outlet C_turbine_out annotation (Placement(transformation(extent={{90,250},{110,270}}), iconTransformation(extent={{90,250},{110,270}})));
  Modelica.Blocks.Interfaces.RealInput VRot "Pump rotational speed" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,-178}), iconTransformation(
        extent={{-9,-9},{9,9}},
        rotation=90,
        origin={1,-249})));
equation
  // Pump variables
  pump.VRotn = VRotn;

  pump.a1 = a1;
  pump.a2 = a2;
  pump.a3 = a3;

  pump.b1 = b1;
  pump.b2 = b2;
  pump.b3 = b3;

  pump.rm = rm; // Mechanical yield is equal to 1 because the turbine already provides mechanical power
  pump.rh = rh;
  pump.rh = rhmin;
  pump.hn = hn;

  // Turbine variables
  turbine.Cst = Cst;
  turbine.eta_is = eta_is;

  // Power equation link
  //turbine.W = pump.Wm; // used connector for debuging

  // Hyp no nozzle / nozzle negligible
  turbine.eta_nz = 1;
  turbine.area_nz = 1;
  connect(C_pump_in, pump.C_in) annotation (Line(points={{100,-140},{56,-140},{56,-120},{10,-120}},
                                                                                                  color={28,108,200}));
  connect(C_pump_out, pump.C_out) annotation (Line(points={{-100,-140},{-56,-140},{-56,-120},{-10,-120}},
                                                                                                        color={28,108,200}));
  connect(C_turbine_in, turbine.C_in) annotation (Line(points={{-60,60},{-36,60},{-36,40},{-10,40}}, color={28,108,200}));
  connect(turbine.C_out, C_turbine_out) annotation (Line(points={{10,40},{46,40},{46,260},{100,260}}, color={28,108,200}));
  connect(pump.VRot, VRot) annotation (Line(points={{0,-132},{0,-178}}, color={0,0,127}));
  connect(turbine.C_W_out, pump.C_power) annotation (Line(points={{10,31.6},{10,-104},{0,-104},{0,-109.2}}, color={244,125,35}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-260,-260},{260,260}}), graphics={
        Rectangle(
          extent={{59,11},{-59,-11}},
          lineThickness=0.5,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          rotation=90,
          origin={1,9}),
        Ellipse(
          extent={{-100,-40},{100,-240}},
          lineColor={0,0,0},
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid),
        Line(points={{-2,-200},{-80,-140}}),
        Line(points={{-80,-140},{80,-140}}),
        Line(points={{0,-80},{-80,-140}}),
                               Polygon(
          points={{-100,60},{-100,40},{-100,-40},{-100,-60},{-80,-66},{80,-100},{100,-100},{100,-80},{100,77.539},{100,100},{80,100},{-80,68},{-100,60}},
          lineColor={63,81,181},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          origin={0,160},
          rotation=90),        Polygon(
          points={{-92,59},{-92,41},{-92,-39},{-92,-53},{-74,-59},{72,-89},{92,-93},{92,-71},{92,71},{92,93},{72,91},{-72,63},{-92,59}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          origin={2,159},
          rotation=90),
        Line(
          points={{0,86},{0,-86}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier,
          origin={0,230},
          rotation=90),
        Line(
          points={{0,78},{0,-78}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier,
          origin={0,182},
          rotation=90),
        Line(
          points={{0,68},{0,-68}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier,
          origin={0,136},
          rotation=90),
        Line(
          points={{0,59},{0,-59}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier,
          origin={0,93},
          rotation=90),
        Rectangle(
          extent={{75,2},{-75,-2}},
          lineThickness=0.5,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          origin={1,160},
          rotation=90)}),                     Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-260,-260},{260,260}})));
end TurboPump;
