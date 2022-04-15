within MetroscopeModelingLibrary.MultiFluid;
package HeatExchangers
  extends MetroscopeModelingLibrary.Icons.HeatExchangePackage;

  model HRSGEconomiser

    import MetroscopeModelingLibrary.Units;
    import MetroscopeModelingLibrary.Units.Inputs;

    Inputs.InputArea S;
    Inputs.InputHeatExchangeCoefficient Kth;

    parameter String QCp_max_side = "cold";
    // Warning :
    // QCp_max_side = cold only for EC BP (aka condensate preheater)
    // Otherwise, flue gases usually correspond to QCp_max_side

    Units.Power W;
    Units.MassFlowRate Q_cold;
    Units.MassFlowRate Q_hot;
    Units.Temperature T_cold_in;
    Units.Temperature T_cold_out;
    Units.Temperature T_hot_in;
    Units.Temperature T_hot_out;

    FlueGases.Connectors.Inlet flueGasesInlet annotation (Placement(
          transformation(extent={{-80,-10},{-60,10}}), iconTransformation(
            extent={{-80,-10},{-60,10}})));
    FlueGases.Connectors.Outlet flueGasesOutlet annotation (
        Placement(transformation(extent={{60,-10},{80,10}}),
          iconTransformation(extent={{60,-10},{80,10}})));
    WaterSteam.Connectors.Outlet waterOutlet annotation (Placement(
          transformation(extent={{-40,60},{-20,80}}), iconTransformation(
            extent={{-40,60},{-20,80}})));
    WaterSteam.Connectors.Inlet waterInlet annotation (Placement(
          transformation(extent={{20,-80},{40,-60}}), iconTransformation(
            extent={{20,-80},{40,-60}})));
    Power.HeatExchange.NTUHeatExchange nTUHeatExchange annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={6,-20})));
  equation

    connect(waterInlet, waterOutlet) annotation (Line(points={{30,-70},{30,70},
            {-30,70}}, color={28,108,200}));
    connect(flueGasesOutlet, flueGasesInlet) annotation (Line(points={{70,0},
            {44,0},{44,-44},{-26,-44},{-26,0},{-70,0}}, color={95,95,95}));
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
  end HRSGEconomiser;
end HeatExchangers;
