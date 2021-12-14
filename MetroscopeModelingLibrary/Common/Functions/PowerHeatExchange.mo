within MetroscopeModelingLibrary.Common.Functions;
function PowerHeatExchange
  extends Modelica.Icons.Function;
  input Modelica.Units.SI.MassFlowRate Q_hot "Hot mass flow rate at the inlet";
  input Modelica.Units.SI.MassFlowRate Q_cold
    "Cold mass flow rate at the inlet";
  input Modelica.Units.SI.SpecificHeatCapacity Cp_hot
    "Hot fluid specific heat capacity";
  input Modelica.Units.SI.SpecificHeatCapacity Cp_cold
    "Cold fluid specific heat capacity";
  input Modelica.Units.SI.Temperature T_hot_in
    "Temperature, hot side, at the inlet";
  input Modelica.Units.SI.Temperature T_cold_in
    "Temperature, cold side, at the inlet";
  input Modelica.Units.SI.CoefficientOfHeatTransfer Kth
    "Global heat transfer coefficient";
  input Modelica.Units.SI.Area S "External exchange surface";
  input Real HotPhase " = 1 or 0 for one-phase flow, in ]0;1[ for two-phase flow - could be associated to mass fraction";
  output Modelica.Units.SI.Power Wth "Heat power transfer";
protected
  Real NTU "Number of transfer units";
  Real QCpMIN "Minimum heat capacity for the two fluids";
  Real QCpMAX "Maximum heat capacity for the two fluids";
  Real Cr "Heat capacity ratio";
  Real epsilon "Exchanger efficiency";
  Modelica.Units.SI.Power Wth_max "max possible heat power transfer";
algorithm
  /* Heat exchanger efficiency calculation*/
  /*one phase flow on cold side*/
  if ((HotPhase > 0) and (HotPhase < 1)) then
    /* Two-phase flow on hot side*/
    QCpMIN:=Q_cold*Cp_cold;
    NTU := Kth*S/QCpMIN;
    epsilon := 1 - exp(-NTU);
  else
    /*one-phase flow on hot side*/
    QCpMIN :=min(Q_hot*Cp_hot, Q_cold*Cp_cold);
    QCpMAX :=max(Q_hot*Cp_hot, Q_cold*Cp_cold);
    Cr :=QCpMIN/QCpMAX;
    NTU := Kth*S/QCpMIN;
    /* Shell-and-tube Heat Exchanger
    formula given p207 in TSP handbook*/
    if QCpMAX==Q_hot*Cp_hot then
      /* QCpMAX is associated to the mixed fluid, shell side, considered as hot side */
      epsilon := (1 - exp(-Cr*(1 - exp(-NTU))))/Cr;
    else
      /* QCpMAX is associated to the unmixed fluid, tube side, considered as cold side */
      epsilon := 1 - exp(-(1 - exp(-Cr*NTU))/Cr);
    end if;
  end if;
  Wth_max :=QCpMIN*(T_hot_in - T_cold_in);
  Wth :=epsilon*Wth_max;
end PowerHeatExchange;
