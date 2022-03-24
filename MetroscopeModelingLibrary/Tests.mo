within MetroscopeModelingLibrary;
package Tests

  package WaterSteamTests

    model WaterFlowModelTest
      WaterSteam.BaseClasses.WaterFlowModel waterFlowModel annotation (Placement(transformation(extent={{-22,-22},{24,24}})));
      WaterSteam.BoundaryConditions.WaterSource waterSource annotation (Placement(transformation(extent={{-82,-20},{-44,18}})));
      WaterSteam.BoundaryConditions.WaterSink waterSink annotation (Placement(transformation(extent={{52,-20},{94,20}})));
    equation
      waterFlowModel.W_input = 0;
      waterFlowModel.DP_input = 0;

      waterSource.Q_out = -100;
      waterSource.h_out = 1e6;
      waterSource.P_out = 1e5;

      assert(abs(waterSink.Q_in + waterSource.Q_out) <= 1e-5, "In flow model, DM should be 0");
      connect(waterFlowModel.C_in, waterSource.C_out) annotation (Line(
          points={{-22,1},{-22,-1},{-54.64,-1}},
          color={28,108,200},
          thickness=0.5));
      connect(waterFlowModel.C_out, waterSink.C_in) annotation (Line(
          points={{24,1},{24,0},{62.08,0}},
          color={28,108,200},
          thickness=1));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor = {75,138,73},
                    fillColor={255,255,255},
                    fillPattern = FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(lineColor = {0,0,255},
                    fillColor = {75,138,73},
                    pattern = LinePattern.None,
                    fillPattern = FillPattern.Solid,
                    points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end WaterFlowModelTest;

    model WaterIsoPFlowModelTest
      WaterSteam.BaseClasses.WaterIsoPFlowModel waterIsoPFlowModel annotation (Placement(transformation(extent={{-22,-22},{24,24}})));
      WaterSteam.BoundaryConditions.WaterSource waterSource annotation (Placement(transformation(extent={{-82,-20},{-44,18}})));
      WaterSteam.BoundaryConditions.WaterSink waterSink annotation (Placement(transformation(extent={{52,-20},{94,20}})));
    equation
      waterIsoPFlowModel.W_input = 0;

      waterSource.Q_out = -100;
      waterSource.h_out = 1e6;
      waterSource.P_out = 1e5;

      assert(abs(waterSink.Q_in + waterSource.Q_out) <= 1e-5, "In IsoPFlowModel, DM should be 0");
      assert(abs(waterSource.P_out - waterSink.P_in) <= 1e-5, "In IsoPFlowModel, DP should be 0");
      connect(waterSource.C_out, waterIsoPFlowModel.C_in) annotation (Line(
          points={{-54.64,-1},{-54.64,1},{-22,1}},
          color={28,108,200},
          thickness=1));
      connect(waterIsoPFlowModel.C_out, waterSink.C_in) annotation (Line(
          points={{24,1},{24,0},{62.08,0}},
          color={28,108,200},
          thickness=1));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor = {75,138,73},
                    fillColor={255,255,255},
                    fillPattern = FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(lineColor = {0,0,255},
                    fillColor = {75,138,73},
                    pattern = LinePattern.None,
                    fillPattern = FillPattern.Solid,
                    points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end WaterIsoPFlowModelTest;
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
            fillColor={78,138,73},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
  end WaterSteamTests;
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
          fillColor={78,138,73},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
end Tests;
