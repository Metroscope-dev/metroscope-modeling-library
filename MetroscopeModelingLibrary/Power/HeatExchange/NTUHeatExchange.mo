within MetroscopeModelingLibrary.Power.HeatExchange;
model NTUHeatExchange

  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Units;

  /* Exchanger configuration and parameters */
  parameter String config = "monophasic_counter_current";
  parameter String QCp_max_side = "cold";
  Inputs.InputArea S "Heat exchange surface";
  Inputs.InputHeatExchangeCoefficient Kth "Heat exchange coefficient";


  /* Exchanger output */
  Units.Power W;

  /* Exchanger boundary conditions */
  Inputs.InputMassFlowRate Q_hot "Hot mass flow rate at the inlet";
  Inputs.InputMassFlowRate Q_cold "Cold mass flow rate at the inlet";
  Inputs.InputHeatCapacity Cp_hot "Hot fluid specific heat capacity";
  Inputs.InputHeatCapacity Cp_cold "Cold fluid specific heat capacity";
  Inputs.InputTemperature T_hot_in "Temperature, hot side, at the inlet";
  Inputs.InputTemperature T_cold_in "Temperature, cold side, at the inlet";

protected
  Real QCpMIN;
  Real QCpMAX;
  Real NTU;
  Real Cr;
  Real epsilon;
  Real W_max;

equation

  W_max = QCpMIN*(T_hot_in - T_cold_in);
  W = epsilon*W_max;

  if config == "monophasic_counter_current" then

    NTU = Kth*S/QCpMIN;
    Cr =QCpMIN/QCpMAX;

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

    Cr = QCpMIN/QCpMAX;
    NTU = Kth*S/QCpMIN;
    epsilon = 1 - exp(-NTU);

  end if;




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
