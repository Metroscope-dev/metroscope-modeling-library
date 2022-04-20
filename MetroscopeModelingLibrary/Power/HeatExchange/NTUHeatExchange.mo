within MetroscopeModelingLibrary.Power.HeatExchange;
model NTUHeatExchange

  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.MassFlowRate Q_hot_0 = 50 "Init parameter for Hot mass flow rate at the inlet";
  parameter Units.MassFlowRate Q_cold_0 = 1500 "Init parameter for Cold mass flow rate at the inlet";
  parameter Units.Temperature T_hot_in_0 = 273.15 + 200 "Init parameter for Hot mass flow rate at the inlet";
  parameter Units.Temperature T_cold_in_0 = 273.15 + 100 "Init parameter for Cold mass flow rate at the inlet";
  parameter Units.HeatCapacity Cp_hot_0 = 45 "Init parameter for Hot fluid specific heat capacity";
  parameter Units.HeatCapacity Cp_cold_0 = 75 "Init parameter for Cold fluid specific heat capacity";
  parameter Units.Area S_0 = 100 "init parameter for Heat exchange surface";
  parameter Units.HeatExchangeCoefficient Kth_0 = 5000 "init parameter for Heat exchange coefficient";

  /* Exchanger configuration and parameters */
  parameter String config = "shell_and_tubes_two_passes";
  parameter String QCp_max_side = "cold";
  Inputs.InputArea S(start=S_0) "Heat exchange surface";
  Inputs.InputHeatExchangeCoefficient Kth(start=Kth_0) "Heat exchange coefficient";

  /* Exchanger output */
  Units.Power W(start=W_0);

  /* Exchanger boundary conditions */
  Inputs.InputMassFlowRate Q_hot(start=Q_hot_0) "Hot mass flow rate at the inlet";
  Inputs.InputMassFlowRate Q_cold(start=Q_cold_0) "Cold mass flow rate at the inlet";
  Inputs.InputHeatCapacity Cp_hot(start=Cp_hot_0) "Hot fluid specific heat capacity";
  Inputs.InputHeatCapacity Cp_cold(start=Cp_cold_0) "Cold fluid specific heat capacity";
  Inputs.InputTemperature T_hot_in(start=T_hot_in_0) "Temperature, hot side, at the inlet";
  Inputs.InputTemperature T_cold_in(start=T_cold_in_0) "Temperature, cold side, at the inlet";

  // Exchange parameters
  Real QCpMIN(unit="W/K", start=QCpMIN_0);
  Real QCpMAX(unit="W/K", start=QCpMAX_0);
  Real NTU(unit="1", start=NTU_0);
  Units.Fraction Cr(start=Cr_0);
  Units.Fraction epsilon(start=epsilon_0);
  Units.Power W_max(start=W_max_0);
protected
  parameter Real QCpMIN_0(unit="W/K") = if QCp_max_side == "hot" then Q_cold_0 * Cp_cold_0 else Q_hot_0 * Cp_hot_0;
  parameter Real QCpMAX_0(unit="W/K") = if QCp_max_side == "cold" then Q_cold_0 * Cp_cold_0 else Q_hot_0 * Cp_hot_0;
  parameter Real NTU_0(unit="1") = Kth_0*S_0/QCpMIN_0;
  parameter Units.Fraction Cr_0 = QCpMIN_0 / QCpMAX_0;
  parameter Units.Fraction epsilon_0 = 2/(1 + Cr_0 + sqrt(1+Cr_0^2)* (1+exp(-NTU_0*(1+Cr_0^2)^0.5))/(1-exp(-NTU_0*(1+Cr_0^2)^0.5)));
  parameter Units.Power W_max_0 = QCpMIN_0*(T_hot_in_0 - T_cold_in_0);
  parameter Units.Power W_0 = epsilon_0*W_max_0;
equation

  W_max = QCpMIN*(T_hot_in - T_cold_in);
  W = epsilon*W_max;
  NTU = Kth*S/QCpMIN;
  Cr = QCpMIN/QCpMAX;

  if config == "shell_and_tubes_two_passes" then

    /* This is a monophasic shell and tube heat exchanger with two tube passes, 
    for instance for U-shaped tubes */

    if QCp_max_side == "hot" then
      QCpMAX = Q_hot*Cp_hot;
      QCpMIN = Q_cold*Cp_cold;
    else
      QCpMIN = Q_hot*Cp_hot;
      QCpMAX = Q_cold*Cp_cold;
    end if;
    assert(QCpMIN < QCpMAX, "QCPMIN is higher than QCpMAX", AssertionLevel.error);

    epsilon = 2*1/( 1 + Cr + sqrt(1+Cr^2)* (1+exp(-NTU*(1+Cr^2)^0.5))/(1-exp(-NTU*(1+Cr^2)^0.5)));


  elseif config == "monophasic_cross_current" then

  /* This is a monophasic shell and tube heat exchanger with cross current flow, which is equivalent
  to saying there is one tube pass, as opposed to the two tube passes of U-shaped tubes */

    if QCp_max_side == "hot" then
      /* QCpMAX is associated to the mixed fluid, shell side, considered as hot side */
      QCpMAX = Q_hot*Cp_hot;
      QCpMIN = Q_cold*Cp_cold;
      assert(QCpMIN < QCpMAX, "QCPMIN is higher than QCpMAX", AssertionLevel.error);
      epsilon =  (1 - exp(-Cr*(1 - exp(-NTU))))/Cr;
    else
      /* QCpMAX is associated to the unmixed fluid, tube side, considered as cold side */
      QCpMIN = Q_hot*Cp_hot;
      QCpMAX = Q_cold*Cp_cold;
      assert(QCpMIN < QCpMAX, "QCPMIN is higher than QCpMAX", AssertionLevel.error);
      epsilon =  1 - exp(-(1 - exp(-Cr*NTU))/Cr);
    end if;


  elseif config == "condenser_counter_current" then

    QCpMAX = Q_hot*Cp_hot;
    QCpMIN = Q_cold*Cp_cold;

    epsilon = 1 - exp(-NTU);



  else // Added this forbidden case to simplify model checking, but it is anyway overriden by assert below
    QCpMAX = 0;
    QCpMIN = 0;

    epsilon = 0;
  end if;

  assert(config=="condenser_counter_current" or config=="shell_and_tubes_two_passes" or config=="monophasic_cross_current", "config parameter of NTUHeatExchange should be one of 'shell_and_tubes_two_passes', 'condenser_counter_current'");

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{-66,-70},{-48,-70},{-48,-64},{-50,-52},{-54,-42},{-58,-32},{-60,
              -26},{-60,-14},{-60,-6},{-54,10},{-52,14},{-50,22},{-48,32},{-48,38},
              {-32,38},{-56,70},{-56,70},{-80,38},{-64,38},{-64,32},{-66,26},{-68,
              18},{-72,10},{-74,4},{-76,-4},{-76,-14},{-76,-26},{-74,-34},{-70,-46},
              {-66,-56},{-66,-70}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-10,-70},{8,-70},{8,-64},{6,-52},{2,-42},{-2,-32},{-4,-26},{-4,
              -14},{-4,-6},{2,10},{4,14},{6,22},{8,32},{8,38},{24,38},{0,70},{0,
              70},{-24,38},{-8,38},{-8,32},{-10,26},{-12,18},{-16,10},{-18,4},{-20,
              -4},{-20,-14},{-20,-26},{-18,-34},{-14,-46},{-10,-56},{-10,-70}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{48,-70},{66,-70},{66,-64},{64,-52},{60,-42},{56,-32},{54,-26},
              {54,-14},{54,-6},{60,10},{62,14},{64,22},{66,32},{66,38},{82,38},{
              58,70},{58,70},{34,38},{50,38},{50,32},{48,26},{46,18},{42,10},{40,
              4},{38,-4},{38,-14},{38,-26},{40,-34},{44,-46},{48,-56},{48,-70}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end NTUHeatExchange;
