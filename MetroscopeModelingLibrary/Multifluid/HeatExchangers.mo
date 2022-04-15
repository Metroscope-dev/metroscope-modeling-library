within MetroscopeModelingLibrary.MultiFluid;
package HeatExchangers
  extends MetroscopeModelingLibrary.Icons.HeatExchangePackage;

  model Economiser

    import MetroscopeModelingLibrary.Units;
    import MetroscopeModelingLibrary.Units.Inputs;

    Inputs.InputArea S;
    Inputs.InputHeatExchangeCoefficient Kth;
    Inputs.InputFrictionCoefficient Kfr_cold;
    Inputs.InputFrictionCoefficient Kfr_hot;
    Inputs.InputTemperature T_cold_in;
    Inputs.InputTemperature T_hot_in;

    parameter String QCp_max_side = "hot";
    // Warning :
    // QCp_max_side = cold only for EC LP (aka condensate preheater)
    // Otherwise, flue gases usually correspond to QCp_max_side
    Units.Temperature nominal_cold_side_temperature_rise; // water reference temperature rise based on H&MB diagramm values
    Units.Temperature nominal_hot_side_temperature_rise; // flue gases reference temperature rise based on H&MB diagramm values
    Units.Power W;
    Units.MassFlowRate Q_cold;
    Units.MassFlowRate Q_hot;

      // Initialization parameters
    parameter Units.MassFlowRate Q_cold_0 = 500;
    parameter Units.MassFlowRate Q_hot_0 = 50;

  protected
    Units.HeatCapacity Cp_cold_min;
    Units.HeatCapacity Cp_cold_max;
    Units.HeatCapacity Cp_hot_min;
    Units.HeatCapacity Cp_hot_max;
    Media.WaterSteamMedium.ThermodynamicState state_cold_out; // estimation of the water outlet thermodynamic state
    Media.FlueGasesMedium.ThermodynamicState state_hot_out; // estimation of the flue gases outlet thermodynamic state

  public
    FlueGases.Connectors.Inlet C_hot_in annotation (Placement(transformation(
            extent={{-80,-10},{-60,10}}), iconTransformation(extent={{-80,-10},{-60,
              10}})));
    FlueGases.Connectors.Outlet C_hot_out annotation (Placement(transformation(
            extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,10}})));
    WaterSteam.Connectors.Inlet C_cold_in annotation (Placement(transformation(
            extent={{20,-80},{40,-60}}), iconTransformation(extent={{20,-80},{40,-60}})));
    WaterSteam.Connectors.Outlet C_cold_out annotation (Placement(transformation(
            extent={{-40,60},{-20,80}}), iconTransformation(extent={{-40,60},{-20,
              80}})));
    FlueGases.Pipes.Pipe hot_side_pipe annotation (Placement(transformation(extent={{-48,18},{-28,38}})));
    Power.HeatExchange.NTUHeatExchange HX(config="monophasic_cross_current",QCp_max_side=QCp_max_side) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={18,10})));
    FlueGases.BaseClasses.IsoPFlowModel hot_side annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-2,8})));
    WaterSteam.BaseClasses.IsoPFlowModel cold_side annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={40,12})));
    WaterSteam.Pipes.Pipe cold_side_pipe annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={40,-26})));
  equation
      // Definitions
    Q_cold =cold_side.Q_in;
    Q_hot =hot_side.Q_in;
    T_cold_in =cold_side.T_in;
    T_hot_in =hot_side.T_in;
    cold_side.W = W;

    // Energy balance
    hot_side.W + cold_side.W = 0;

    // Pressure losses
    cold_side_pipe.delta_z=0;
    cold_side_pipe.Kfr = Kfr_cold;
    hot_side_pipe.delta_z=0;
    hot_side_pipe.Kfr = Kfr_hot;

    // Power Exchange
    HX.W = W;
    HX.Kth =  Kth;
    HX.S = S;
    HX.Q_cold = Q_cold;
    HX.Q_hot = Q_hot;
    HX.T_cold_in = T_cold_in;
    HX.T_hot_in = T_hot_in;
    HX.Cp_cold = (Cp_cold_min + Cp_cold_max)/2;
    HX.Cp_hot = (Cp_hot_min + Cp_hot_max)/2;

    // For each medium, an average Cp is calculated beteween Cp inlet and an estimation of Cp outlet.
    // The estimation of the Cp outlet is calculated for an outlet temperature based on the nominal temperature rise of the H&MB diagram.
    // For more details about this hypothesis, please refer the Economiser page of the MML documentation.

    Cp_cold_min =MetroscopeModelingLibrary.Media.WaterSteamMedium.specificHeatCapacityCp(cold_side.state_in); // water steam inlet Cp
    state_cold_out = MetroscopeModelingLibrary.Media.WaterSteamMedium.setState_pTX(cold_side.P_in, cold_side.T_in + nominal_cold_side_temperature_rise,cold_side.Xi_in);
    Cp_cold_max= MetroscopeModelingLibrary.Media.WaterSteamMedium.specificHeatCapacityCp(state_cold_out); // water steam outlet Cp

    Cp_hot_max=MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(hot_side.state_in);// fg inlet Cp
    state_hot_out = MetroscopeModelingLibrary.Media.FlueGasesMedium.setState_pTX(hot_side.P_in, hot_side.T_in + nominal_hot_side_temperature_rise,hot_side.Xi_in);
    Cp_hot_min =MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(state_hot_out);  // fg outlet Cp

    connect(cold_side_pipe.C_out,cold_side. C_in)
      annotation (Line(points={{40,-16},{40,2}},  color={28,108,200}));
    connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{40,-36},{40,
            -54},{40,-70},{30,-70}}, color={28,108,200}));
    connect(cold_side.C_out, C_cold_out)
      annotation (Line(points={{40,22},{40,70},{-30,70}}, color={28,108,200}));
    connect(hot_side.C_out, C_hot_out) annotation (Line(points={{-2,-2},{-2,-4},{70,
            -4},{70,0}}, color={95,95,95}));
    connect(C_hot_in, hot_side_pipe.C_in)
      annotation (Line(points={{-70,0},{-70,28},{-48,28}}, color={95,95,95}));
    connect(hot_side_pipe.C_out, hot_side.C_in)
      annotation (Line(points={{-28,28},{-2,28},{-2,18}}, color={95,95,95}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-68,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid), Line(
            points={{30,-72},{30,74},{0,74},{0,-74},{-30,-74},{-30,70}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end Economiser;
end HeatExchangers;
