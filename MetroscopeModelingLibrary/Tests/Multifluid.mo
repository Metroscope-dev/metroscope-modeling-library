within MetroscopeModelingLibrary.Tests;
package Multifluid


  package Machines



    model CombustionChamber_direct


      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=17e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=0.7e6) "J/kg";
      input Units.OutletMassFlowRate source_Q(start=-500) "kg/s";

      input Units.Pressure P_fuel(start = 30e5);
      input Units.SpecificEnthalpy h_fuel(start=0.9e6);
      input Units.OutletMassFlowRate Q_fuel(start=15);

      // Parameters
      parameter Units.SpecificEnthalpy LHV = 48130e3;
      parameter Units.DifferentialPressure combustion_chamber_pressure_loss = 0.1e5;

      MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source_fuel annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,-38})));
      MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source_air annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
      MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink_exhaust annotation (Placement(transformation(extent={{28,-10},{48,10}})));
    equation

      // Boundary conditions
      source_air.P_out = source_P;
      source_air.h_out = source_h;
      source_air.Q_out = source_Q;
      source_air.Xi_out = {0.768,0.232,0.0,0.0,0.0};

      source_fuel.P_out = P_fuel;
      source_fuel.h_out = h_fuel;
      source_fuel.Q_out = - Q_fuel;
      source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

      // Parameters
      combustionChamber.LHV = LHV;
      combustionChamber.DP = combustion_chamber_pressure_loss;

      connect(combustionChamber.inlet1, source_fuel.C_out) annotation (Line(points={{0,-10},{0,-33},{2.77556e-16,-33}}, color={213,213,0}));
      connect(combustionChamber.inlet, source_air.C_out) annotation (Line(points={{-10,0},{-33,0}}, color={95,95,95}));
      connect(combustionChamber.outlet, sink_exhaust.C_in) annotation (Line(points={{10,0},{33,0}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor = {75,138,73},
                    fillColor={255,255,255},
                    fillPattern = FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(
              origin={20,14},
              lineColor={78,138,73},
              fillColor={95,95,95},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
            Polygon(
              origin={20,14},
              lineColor={78,138,73},
              fillColor={213,213,0},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
            Polygon(
              origin={20,14},
              lineColor={78,138,73},
              fillColor={28,108,200},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}),    Diagram(coordinateSystem(preserveAspectRatio=false)));
    end CombustionChamber_direct;
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
            radius=25.0),
          Polygon(
            origin={8,14},
            lineColor={78,138,73},
            fillColor={95,95,95},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
          Polygon(
            origin={8,14},
            lineColor={78,138,73},
            fillColor={213,213,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
          Polygon(
            origin={8,14},
            lineColor={78,138,73},
            fillColor={28,108,200},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}));
  end Machines;
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
          radius=25.0),
        Polygon(
          origin={8,14},
          lineColor={78,138,73},
          fillColor={95,95,95},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
        Polygon(
          origin={8,14},
          lineColor={78,138,73},
          fillColor={213,213,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
        Polygon(
          origin={8,14},
          lineColor={78,138,73},
          fillColor={28,108,200},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}));
end Multifluid;
