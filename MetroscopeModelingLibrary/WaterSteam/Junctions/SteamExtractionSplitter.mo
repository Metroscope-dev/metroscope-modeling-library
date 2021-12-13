within MetroscopeModelingLibrary.WaterSteam.Junctions;
model SteamExtractionSplitter
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;

  connector InputReal = input Real;

  Modelica.Units.SI.MassFlowRate Q_in(start=4000) "Inlet Mass flow rate";
  Modelica.Units.SI.AbsolutePressure P_in(start=71e5) "Inlet Pressure";
  Modelica.Units.SI.SpecificEnthalpy h_in(start=1e5) "Inlet specific enthalpy";
  Modelica.Units.SI.SpecificEnthalpy hesat(start=2e6)
    "Enthalpy of saturated water";
  Modelica.Units.SI.SpecificEnthalpy hvsat(start=1e6)
    "Enthalpy of saturated vapor";
  Modelica.Units.SI.MassFraction x_ext_out(start=0.8)
    "Vapor mass fraction at extraction outlet (0 <= x_ext_out <= x_in)";
  Modelica.Units.SI.MassFraction x_in(start=0.8) "Vapor mass fraction at inlet";
  Modelica.Units.SI.MassFraction x_main_out(start=0.8)
    "Vapor mass fraction at main outlet";
  InputReal alpha(start = 1) "Extraction paramater";
  replaceable Common.Partial.BasicTransportModel extractedFlow(redeclare
      package Medium = WaterSteamMedium) annotation (Placement(transformation(
        extent={{19.5,-8.5},{-19.5,8.5}},
        rotation=90,
        origin={-0.5,-60.5})));
  Common.Connectors.FluidOutlet C_ext_out(redeclare package Medium =
        WaterSteamMedium) annotation (Placement(transformation(extent={{-10,
            -110},{10,-90}}),
                        iconTransformation(extent={{-10,-110},{10,-90}})));
  replaceable Common.Partial.BasicTransportModel mainFlow(redeclare package
      Medium = WaterSteamMedium) annotation (Placement(transformation(
        extent={{-20,-8},{20,8}},
        rotation=0,
        origin={54,-30})));
  Common.Connectors.FluidOutlet C_main_out(redeclare package Medium =
        WaterSteamMedium) annotation (Placement(transformation(extent={{94,-50},
            {114,-30}}), iconTransformation(extent={{94,-50},{114,-30}})));
  Common.Connectors.FluidInlet C_in(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-108,-50},{-88,-30}}),
        iconTransformation(extent={{-108,-50},{-88,-30}})));
equation
  // Defenition of all intermediate variables
  hvsat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_in));
  hesat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_in));
  Q_in =extractedFlow.Q_in + mainFlow.Q_in;
  h_in * Q_in =extractedFlow.h_in*extractedFlow.Q_in + mainFlow.h_in*mainFlow.Q_in;
  x_in = (h_in - hesat)/(hvsat-hesat);
  // Mass balance
  mainFlow.Q_in + mainFlow.Q_out = 0;
  extractedFlow.Q_in + extractedFlow.Q_out = 0;
  //Energy balance
  h_in * Q_in + mainFlow.Q_out*mainFlow.h_out + extractedFlow.Q_out*extractedFlow.h_out = 0;
  // Saturation
  mainFlow.h_out = x_main_out*hvsat + (1 - x_main_out)*hesat;
  extractedFlow.h_out = x_ext_out*hvsat + (1 - x_ext_out)*hesat;
  x_ext_out=alpha*x_in;
  //Mechanical Balance
  P_in =mainFlow.P_out;
  P_in =extractedFlow.P_out;
  P_in =extractedFlow.P_in;
  connect(mainFlow.C_out, C_main_out)
    annotation (Line(points={{74.4,-30},{94,-30},{94,-40},{104,-40}},
                                                    color={238,46,47}));
  connect(mainFlow.C_in, C_in)
    annotation (Line(points={{34,-30},{-34,-30},{-34,-40},{-98,-40}},
                                                   color={0,0,255}));
  connect(extractedFlow.C_in, C_in) annotation (Line(points={{-0.5,-41},{-0.5,
          -40},{-98,-40}},
                       color={0,0,255}));
  connect(extractedFlow.C_out, C_ext_out) annotation (Line(points={{-0.5,-80.39},
          {-0.5,-86.195},{0,-86.195},{0,-100}}, color={238,46,47}));
  connect(C_in, C_in) annotation (Line(points={{-98,-40},{-98,-40},{-98,-40},{
          -98,-40}}, color={63,81,181}));
  connect(C_ext_out, C_ext_out) annotation (Line(points={{0,-100},{-5,-100},{-5,
          -100},{0,-100}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,-20}}), graphics={Polygon(
          points={{-100,-20},{-100,-60},{-46,-60},{-8,-100},{10,-100},{-16,-60},
              {100,-60},{100,-20},{-100,-20}},
          lineColor={64,82,185},
          fillColor={236,238,248},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,-20}})),
    Documentation(info="<html>
<p><b>V1</b> Creation of the component and the single test (03/05/2019)</p>
<p><br><b>Parameters</b> :</p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p>Symbol</p></td>
<td><p>Meaning</p></td>
<td><p>Unit</p></td>
</tr>
<tr>
<td><p>alpha</p></td>
<td><p>Extraction parameter that links vapor mass fractions at the inlet and at the extraction according the following equations : <span style=\"font-family: Courier New;\">x_ext_out=alpha*x_in</span></p></td>
<td><p>-</p></td>
</tr>
</table>
<p><br><h4>Direct model :</h4></p>
<p>Fixed : alpha</p>
<p>As boundary conitions : Q_in, P_in, h_in and extractedFlow.Q_out</p>
<p>output : x_ext, x_main_out, mainFlow.Q_out, mainFlow.h_out, extractedFlow.h_out</p>
<p><b>Modelling choices</b> :</p>
<p>- No pressure loss in the component</p>
</html>"));
end SteamExtractionSplitter;
