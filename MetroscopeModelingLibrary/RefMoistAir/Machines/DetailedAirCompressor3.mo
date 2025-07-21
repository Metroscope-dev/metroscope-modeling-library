within MetroscopeModelingLibrary.RefMoistAir.Machines;
model DetailedAirCompressor3

  extends MetroscopeModelingLibrary.RefMoistAir.BaseClasses.FlowModelModifAH(Q_0 = 500, rho_0 = 1)
                                                                                           annotation (IconMap(primitivesVisible=false));
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputReal tau(start=15, min = 1) "Compression rate";
  Inputs.InputReal eta_is(start=0.8, min=0, max=1) "Nominal isentropic efficiency";

  Units.SpecificEnthalpy h_is(start=1e6) "Isentropic compression outlet enthalpy";
  RefMoistAirMedium.ThermodynamicState state_is "Isentropic compression outlet thermodynamic state";
  Real Q_reduced "Compressor reduced mass flow";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage eta_is_decrease(min = 0, max=100) "percentage decrease of eta_is";
  Units.Percentage tau_decrease(min = 0, max=100) "percentage decrease of tau";

  // Added
  Real gamma_in; // gamma calculated at inlet of the compressor
  Real gamma_out; // gamma calculated at outlet of the compressor
  Real gamma; // average gamma
  parameter Integer nb_stages = 15;
  parameter Integer nb_extractions = 2;
  parameter Real Q_extract[nb_extractions]; // No default value bc table depends on nb_extractions value
  parameter Integer loc_stage[nb_extractions]; // No default value bc table depends on nb_extractions value
  Integer loc_stage_ext[nb_extractions + 2]; // This table as juste a practicity utility. It is used because 0-indexing is not permitted in Modelica
  Real eta_poly;
  Real T_out_calcul;
  Real deltaT_stage;

  Real tau1;
  Units.Pressure P_extract;
  Real T_rise;
  Real T_extract;
  Real Tinlet;

  import MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink;

  Power.Connectors.Inlet C_W_in annotation (Placement(transformation(extent={{90,50},{110,70}}),  iconTransformation(extent={{90,50},{110,70}})));
  RefMoistAir.Connectors.Outlet C_extract1 annotation (
    Placement(transformation(extent={{-70,62},{-50,62}}), iconTransformation(extent={{-70,42},{-50,62}})));
  RefMoistAir.Connectors.Outlet C_extract2 annotation (
    Placement(transformation(extent={{-40,47},{-20,57}}), iconTransformation(extent={{-40,37},{-20,57}})));
  RefMoistAir.Connectors.Outlet C_extract3 annotation (
    Placement(transformation(extent={{-10,30},{10,50}}), iconTransformation(extent={{-10,30},{10,50}})));
  RefMoistAir.Connectors.Outlet C_extract4 annotation (
    Placement(transformation(extent={{20,15},{40,45}}), iconTransformation(extent={{20,25},{40,45}})));
  RefMoistAir.Connectors.Outlet C_extract5 annotation (
    Placement(transformation(extent={{50,0},{70,40}}),  iconTransformation(extent={{50,20},{70,40}})));
//  Sink dummySink1 annotation(HideResult=true);
//  Sink dummySink2 annotation(HideResult=true);
//  Sink dummySink3 annotation(HideResult=true);

algorithm

  // By default all extraction flow rates are set to 0
  C_extract1.Q :=0;
  C_extract1.Xi_outflow := C_in.Xi_outflow;
  C_extract2.Q :=0;
  C_extract2.Xi_outflow := C_in.Xi_outflow;
  C_extract3.Q :=0;
  C_extract3.Xi_outflow := C_in.Xi_outflow;
  C_extract4.Q :=0;
  C_extract4.Xi_outflow := C_in.Xi_outflow;
  C_extract5.Q :=0;
  C_extract5.Xi_outflow := C_in.Xi_outflow;

  // Prepare the use of loc_stage_ext
  loc_stage_ext[1] := 0;
  for i in 1:nb_extractions loop
    loc_stage_ext[i + 1] := loc_stage[i];
  end for;
  loc_stage_ext[nb_extractions + 2] := nb_stages;

  T_extract := T_in;
  P_extract := P_in;

  for i in 1:(nb_extractions+1) loop
    T_rise := (loc_stage_ext[i+1] - loc_stage_ext[i]) * deltaT_stage;
    Tinlet := T_extract;
    T_extract := T_extract + T_rise;
    tau1 := (1 + T_rise/Tinlet)^(eta_poly*gamma_in/(gamma_in - 1));
    P_extract := P_extract * tau1;

    if i <= nb_extractions then
      if i == 1 then
        C_extract1.Q := -Q_extract[i];
        C_extract1.P := P_extract;
        C_extract1.h_outflow := Medium.specificEnthalpy_pTX(
          P_extract,
          T_extract,
          C_in.Xi_outflow);
      elseif i == 2 then
        C_extract2.Q := -Q_extract[i];
        C_extract2.P := P_extract;
        C_extract2.h_outflow := Medium.specificEnthalpy_pTX(
          P_extract,
          T_extract,
          C_in.Xi_outflow);
      elseif i == 3 then
        C_extract3.Q := -Q_extract[i];
        C_extract3.P := P_extract;
        C_extract3.h_outflow := Medium.specificEnthalpy_pTX(
          P_extract,
          T_extract,
          C_in.Xi_outflow);
      elseif i == 4 then
        C_extract4.Q := -Q_extract[i];
        C_extract4.P := P_extract;
        C_extract4.h_outflow := Medium.specificEnthalpy_pTX(
          P_extract,
          T_extract,
          C_in.Xi_outflow);
      elseif i == 5 then
        C_extract5.Q := -Q_extract[i];
        C_extract5.P := P_extract;
        C_extract5.h_outflow := Medium.specificEnthalpy_pTX(
          P_extract,
          T_extract,
          C_in.Xi_outflow);
      end if;
    end if;
  end for;

equation

  /*
  if nb_extractions < 3 then
    connect(C_extract3, dummySink.C_in);
  end if;
  if nb_extractions < 4 then
    connect(C_extract4, dummySink.C_in);
  end if;
  if nb_extractions < 5 then
    connect(C_extract5, dummySink.C_in);
  end if;
  */

  //connect(C_extract3, dummySink1.C_in);
  //connect(C_extract4, dummySink2.C_in);
  //connect(C_extract5, dummySink3.C_in);

  // Tester avec l'autre formulation equivalente de eta_is en fonction de eta_poly pour voir si on retombe sur la meme chose
  eta_poly = ((gamma_in-1)/gamma_in)*log(tau)/log((tau^((gamma_in-1)/gamma_in)-1)/eta_is+1);
  //T_out_calcul = T_in * tau^((gamma_in-1)/(gamma_in*eta_poly)); // Regression bc of the gap observed on T_out_calcul VS CTDA / - (0.0183*Q+1.6425)
  deltaT_stage = (T_out - T_in)/nb_stages;

  // Conservation equations
  C_out.Q = - ( Q + C_extract1.Q + C_extract2.Q + C_extract3.Q + C_extract4.Q + C_extract5.Q);
  W = Q * (h_out - h_in) + C_extract1.Q * (h_out - C_extract1.h_outflow) + C_extract2.Q * (h_out - C_extract2.h_outflow) + C_extract3.Q * (h_out - C_extract3.h_outflow) + C_extract4.Q * (h_out - C_extract4.h_outflow) + C_extract5.Q * (h_out - C_extract5.h_outflow);

  // Failure modes
  if not faulty then
    eta_is_decrease = 0;
    tau_decrease = 0;
  end if;

  /* Compression ratio */
  tau*(1-tau_decrease/100) = P_out/P_in;

  /* Fluid specific enthalpy after the expansion */
  DH*eta_is*(1-eta_is_decrease/100) = h_is - h_in;

  /* Mechanical power from the turbine */
  C_W_in.W = W;

  /* Isentropic compression */
  state_is =  Medium.setState_psX(P_out, Medium.specificEntropy(state_in), Xi);
  h_is = Medium.specificEnthalpy(state_is);

  // Gamma calculation
  gamma_in = Medium.specificHeatCapacityCp(state_in)/Medium.specificHeatCapacityCv(state_in);
  gamma_out = Medium.specificHeatCapacityCp(state_out)/Medium.specificHeatCapacityCv(state_out);
  gamma = (gamma_in + gamma_out)/2;
  // Ameliorer la definition de gamma, peut etre en definissant un nouveau gamma a chaque entree de pseudo compresseur
  //gamma = (pseudoAirCompressor1.gamma + pseudoAirCompressor2.gamma)/2;

  /* Output variable */
  Q_reduced = Q * sqrt(T_in) / P_in;

  /* Warning */
  assert(x_liq_out < 1e-10, "Condensed water at the outlet of the compressor", AssertionLevel.warning);

  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-80},{100,80}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-80},{100,80}},
        grid={2,2}), graphics={Polygon(
          points={{100,26},{100,14},{100,-14},{100,-26},{80,-32},{-80,-60},{-100,-64},{-100,-40},{-100,40},{-100,64},{-80,60},{80,30},{100,26}},
          lineColor={0,127,127},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{92,20},{92,14},{92,-14},{92,-20},{76,-26},{-72,-50},{-92,-54},{-92,-40},{-92,40},{-92,54},{-70,50},{76,24},{92,20}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={0,160,160},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,127,127}),
        Line(
          points={{-66,38},{-66,-38}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{6,26},{6,-26}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{24,22},{24,-22}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{56,19},{56,-19}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-30,32},{-30,-32}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Polygon(
          points={{-71,-2.5},{-65,-2.5},{-45,-2.5},{0,-7.5},{62,-7.5},{72,-7.5},{72,0.5},{72,0.5},{72,6.5},{63,7.5},{0,6.5},{-45,1.5},{-65,1.5},{-71,1.5},{-71,-0.5},{-71,-0.5},{-71,-2.5}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{40,20.5},{40,-20.5}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-12,29.25},{-12,-29.25}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-48,35.5},{-48,-35.5}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier)}));
end DetailedAirCompressor3;
