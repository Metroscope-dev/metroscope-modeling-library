within MetroscopeModelingLibrary.WaterSteam;
package Pipes

  model SteamExtractionSplitter
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    import MetroscopeModelingLibrary.Units;
    import MetroscopeModelingLibrary.Units.Inputs;

    import MetroscopeModelingLibrary.WaterSteam.Connectors;

    // Initialization parameters
    parameter Units.InletMassFlowRate Q_in_0 = 4000;
    parameter Units.InletMassFlowRate Q_main_0 = 3900;
    parameter Units.InletMassFlowRate Q_ext_0 = Q_in_0 - Q_main_0;
    parameter Units.Pressure P_0 = 71e5;
    parameter Units.SpecificEnthalpy h_in_0 = 1e5;

    // Variables
    Units.InletMassFlowRate Q_in(start=Q_in_0) "Inlet Mass flow rate";
    Units.Pressure P(start=P_0) "Inlet Pressure";
    Units.SpecificEnthalpy h_in(start=h_in_0) "Inlet specific enthalpy";
    Units.SpecificEnthalpy hesat(start=hesat_0) "Enthalpy of saturated water";
    Units.SpecificEnthalpy hvsat(start=hvsat_0) "Enthalpy of saturated vapor";
    Units.MassFraction x_ext_out(start=0.8) "Vapor mass fraction at extraction outlet (0 <= x_ext_out <= x_in)";
    Units.MassFraction x_main_out(start=0.8) "Vapor mass fraction at main outlet";

    Units.MassFraction x_in(start=0.8) "Vapor mass fraction at inlet";
    Inputs.InputReal alpha(start=1, min=0, max=1) "Extraction paramater";

    // Components
    WaterSteam.BaseClasses.WaterIsoPHFlowModel inletFlow(Q_0=Q_in_0, P_0=P_0, h_in_0=h_in_0) annotation (Placement(transformation(extent={{-81,-27},{-31,27}})));
    //WaterSteam.BaseClasses.WaterIsoPFlowModel inletFlow(Q_0=Q_in_0, P_0=P_0, h_in_0=h_in_0) annotation (Placement(transformation(extent={{-81,-27},{-31,27}})));
    WaterSteam.BaseClasses.WaterIsoPFlowModel extractedFlow(Q_0=Q_ext_0, P_0=P_0) annotation (Placement(transformation(
          extent={{-11.5,-10.5},{11.5,10.5}},
          rotation=270,
          origin={0,-30})));
    WaterSteam.BaseClasses.WaterIsoPFlowModel mainFlow(Q_0=Q_main_0, P_0=P_0) annotation (Placement(transformation(extent={{31,-27},{81,27}})));

    Connectors.WaterOutlet C_main_out(Q(start=Q_main_0), P(start=P_0)) annotation (Placement(transformation(extent={{96,-10},{116,10}}), iconTransformation(extent={{96,-10},{116,10}})));
    Connectors.WaterInlet C_in(Q(start=Q_in_0), P(start=P_0)) annotation (Placement(transformation(extent={{-116,-10},{-96,10}}), iconTransformation(extent={{-116,-10},{-96,10}})));
    Connectors.WaterOutlet C_ext_out(Q(start=-Q_ext_0), P(start=P_0)) annotation (Placement(transformation(extent={{-10,-74},{10,-54}}), iconTransformation(extent={{-10,-78},{10,-58}})));
  protected
    parameter Units.SpecificEnthalpy hvsat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_0));
    parameter Units.SpecificEnthalpy hesat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_0));
  equation
    // Definition of all intermediate variables
    hvsat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P));
    hesat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P));
    Q_in = inletFlow.Q;
    P = inletFlow.P;
    h_in = inletFlow.h_in;

    //Energy balance
    //inletFlow.W + extractedFlow.W + mainFlow.W = 0;
    extractedFlow.W + mainFlow.W = 0;

    // Saturation
    x_ext_out = alpha * x_in;

    // Mass Fractions Computation
    x_in = (h_in - hesat)/(hvsat-hesat);
    x_main_out = (mainFlow.h_out - hesat)/(hvsat-hesat);
    x_ext_out = (extractedFlow.h_out - hesat)/(hvsat-hesat);
    connect(inletFlow.C_out, mainFlow.C_in) annotation (Line(points={{-31,-0.27},{31,-0.27}},                         color={28,108,200}));
    connect(extractedFlow.C_in, mainFlow.C_in) annotation (Line(points={{-0.105,-18.5},{0,-18.5},{0,-0.27},{31,-0.27}},    color={28,108,200}));
    connect(extractedFlow.C_out, C_ext_out) annotation (Line(points={{-0.105,-41.5},{-0.105,-54},{0,-54},{0,-64}}, color={28,108,200}));
    connect(C_in, inletFlow.C_in) annotation (Line(points={{-106,0},{-93.5,0},{-93.5,-0.27},{-81,-0.27}}, color={28,108,200}));
    connect(C_main_out, mainFlow.C_out) annotation (Line(points={{106,0},{93.5,0},{93.5,-0.27},{81,-0.27}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,80}}),
                           graphics={Polygon(
            points={{-100,20},{-100,-20},{-46,-20},{-8,-60},{10,-60},{-16,-20},{100,-20},{100,20},{-100,20}},
            lineColor={64,82,185},
            fillColor={236,238,248},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5)}),                                  Diagram(
          coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
          Text(
            extent={{40,-16},{82,-26}},
            textColor={28,108,200},
            textString="main"),
          Text(
            extent={{-18,4},{18,-4}},
            textColor={28,108,200},
            origin={-12,-30},
            rotation=270,
            textString="extracted"),
          Text(
            extent={{-74,-18},{-32,-28}},
            textColor={28,108,200},
            textString="inlet")}));
  end SteamExtractionSplitter;

  model WaterPipe
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    extends Partial.Pipes.Pipe(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                               redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                               redeclare package Medium = WaterSteamMedium);
  end WaterPipe;

  model WaterControlValve
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    extends Partial.Pipes.ControlValve(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                            redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                            redeclare package Medium = WaterSteamMedium);
  end WaterControlValve;

  model WaterHeatLoss
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    extends Partial.Pipes.HeatLoss(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                        redeclare package Medium = WaterSteamMedium);
  end WaterHeatLoss;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),        Rectangle(
          extent={{-48,33},{48,-37}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,11},{-36,-15}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{36,10},{60,-14}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Pipes;
