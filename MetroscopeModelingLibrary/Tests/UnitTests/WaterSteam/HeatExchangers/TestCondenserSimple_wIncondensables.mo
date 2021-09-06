within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.HeatExchangers;
model TestCondenserSimple_wIncondensables
  import MetroscopeModelingLibrary;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet
    annotation (Placement(transformation(extent={{-30,22},{-10,42}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cooling_outlet
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.CondenserSimple_wIncondensables
                                                                      condenser
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink extracted_water
    annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cooling_inlet
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
equation

  // Forward causality
  // The outputs are the temperature of the outlet cooling water, and the flow rate needed from the cooling inlet.

  turbine_outlet.h_out = 1500e3;
  turbine_outlet.Q_out = -150;
  turbine_outlet.P_out = 0.19e5; // You can either give the pressure or temperature, since the water is diphasic.

  cooling_inlet.P_out = 5e5;
  cooling_inlet.T_out = 15+273.15;

  cooling_outlet.h_vol = 1e6;
  extracted_water.h_vol = 1e6;

  condenser.S = 100;
  condenser.WaterHeight = 2;
  condenser.Kfr_cold = 1;
  condenser.Kth = 50000;

  condenser.C_incond = 0.01;
  condenser.P_offset = 0.001e5; //1mbar


  // Reverse Causality
  // Determine the Kth of the condenser by giving either the flow rate in the cooling outlet, or the temperature of the outlet cooling water.
  // Determine Kfr_cold by giving the pressure at the cooling outlet.
  /*
  turbine_outlet.h_out = 1500e3;
  turbine_outlet.Q_out = -150;
  turbine_outlet.P_out = 0.19e5; // You can either give the pressure or temperature, since the water is diphasic.

  cooling_inlet.P_out = 5e5;
  cooling_inlet.T_out = 15+273.15;

  condenser.Tsat = 58.3+273.15; // Give Tsat to determine incondensable pressure and heat exchange coefficient
  condenser.C_incond = 0; // C_incond is put to 0 during calibration, P_offset = P_incond
  // The coefficient C_incond is then decided by looking at the dependancy of P_incond to Tsat

  //cooling_outlet.T_in = 27+273.15;
  cooling_outlet.Q_in = 4500;
  cooling_outlet.P_in = 4e5;

  cooling_outlet.h_vol = 1e6;
  extracted_water.h_vol = 1e6;

  condenser.S = 100;
  condenser.WaterHeight = 2;
  */



  connect(turbine_outlet.C_out, condenser.C_hot_in)
    annotation (Line(points={{-10,32},{0,32},{0,10.1818}},color={238,46,47}));
  connect(condenser.C_hot_out, extracted_water.C_in) annotation (Line(points={{0,-10},
          {0,-30},{10,-30}},         color={238,46,47}));
  connect(cooling_inlet.C_out, condenser.C_cold_in)
    annotation (Line(points={{-30,0},{-20,0},{-20,0.909091},{-10,0.909091}},
                                               color={238,46,47}));
  connect(condenser.C_cold_out, cooling_outlet.C_in)
    annotation (Line(points={{10,0.909091},{20,0.909091},{20,0},{30,0}},
                                            color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-40},
            {60,40}})),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-60,-40},{60,40}})));
end TestCondenserSimple_wIncondensables;
