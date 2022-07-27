within MetroscopeModelingLibrary.Power.HeatExchange;
model LMTDHeatExchange
  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.Temperature T_hot_in_0 = 273.15 + 200 "Init parameter for Hot mass flow rate at the inlet";
  parameter Units.Temperature T_cold_in_0 = 273.15 + 100 "Init parameter for Cold mass flow rate at the inlet";
  parameter Units.Temperature T_hot_out_0 = 273.15 + 200 "Init parameter for Hot mass flow rate at the outlet";
  parameter Units.Temperature T_cold_out_0 = 273.15 + 100 "Init parameter for Cold mass flow rate at the outlet";
  parameter Units.Area S_0 = 100 "init parameter for Heat exchange surface";
  parameter Units.HeatExchangeCoefficient Kth_0 = 5000 "init parameter for Heat exchange coefficient";

  /* Exchanger configuration and parameters */
  //parameter String config = "LMTD_monophasic_counter_current"; No need for parameter as long as there is only one configuration !!
  Inputs.InputArea S(start=S_0) "Heat exchange surface";
  Inputs.InputHeatExchangeCoefficient Kth(start=Kth_0) "Heat exchange coefficient";

  /* Exchanger output */
  Units.Power W(start=1e4);

  /* Exchanger boundary conditions */
  Inputs.InputTemperature T_hot_in(start=T_hot_in_0) "Temperature, hot side, at the inlet";
  Inputs.InputTemperature T_cold_in(start=T_cold_in_0) "Temperature, cold side, at the inlet";
  Inputs.InputTemperature T_hot_out(start=T_hot_out_0) "Temperature, hot side, at the outlet";
  Inputs.InputTemperature T_cold_out(start=T_cold_out_0) "Temperature, cold side, at the outlet";

  // intermediate variables
  Real dT_a(start = 15);
  Real dT_b(start=1);
equation

   dT_a = T_hot_in - T_cold_out;
   dT_b = T_hot_out - T_cold_in;

   // Log mean equation written in an exponential way
   0 = - dT_a + dT_b * exp(Kth*S*(dT_a - dT_b)/W);


  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{-66,-72},{-48,-72},{-48,-66},{-50,-54},{-54,-44},{-58,-34},{-60,-28},{-60,-16},{-60,-8},{-54,8},{-52,12},{-50,20},{-48,30},{-48,36},{-32,36},{-56,68},{-56,68},{-80,36},{-64,36},{-64,30},{-66,24},{-68,16},{-72,8},{-74,2},{-76,-6},{-76,-16},{-76,-28},{-74,-36},{-70,-48},{-66,-58},{-66,-72}},
          lineColor={255,170,255},
          fillColor={255,170,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-10,-72},{8,-72},{8,-66},{6,-54},{2,-44},{-2,-34},{-4,-28},{-4,-16},{-4,-8},{2,8},{4,12},{6,20},{8,30},{8,36},{24,36},{0,68},{0,68},{-24,36},{-8,36},{-8,30},{-10,24},{-12,16},{-16,8},{-18,2},{-20,-6},{-20,-16},{-20,-28},{-18,-36},{-14,-48},{-10,-58},{-10,-72}},
          lineColor={255,170,255},
          fillColor={255,170,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{48,-72},{66,-72},{66,-66},{64,-54},{60,-44},{56,-34},{54,-28},{54,-16},{54,-8},{60,8},{62,12},{64,20},{66,30},{66,36},{82,36},{58,68},{58,68},{34,36},{50,36},{50,30},{48,24},{46,16},{42,8},{40,2},{38,-6},{38,-16},{38,-28},{40,-36},{44,-48},{48,-58},{48,-72}},
          lineColor={255,170,255},
          fillColor={255,170,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end LMTDHeatExchange;
