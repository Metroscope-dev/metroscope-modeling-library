within MetroscopeModelingLibrary;
package MoistAir

  package BoundaryConditions

    model MoistAirSource
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.BoundaryConditions.FluidSource(h_out_0=300, Xi_out_0={MoistAirMedium.massFraction_pTphi(P_out_0, T_out_0, relative_humidity_0)},
                                                     redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
                                                     redeclare package Medium = MoistAirMedium);

      parameter Real relative_humidity_0(min=0, max=1) = 0.1;
      Real relative_humidity(start=relative_humidity_0, min=0, max=1);
    equation
      Xi_out[1] = MoistAirMedium.massFraction_pTphi(P_out, T_out, relative_humidity);
      annotation (
         Diagram(coordinateSystem(preserveAspectRatio=true)));
    end MoistAirSource;

    model MoistAirSink
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.BoundaryConditions.FluidSink(h_in_0=1e3,
                                              redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
                                              redeclare package Medium = MoistAirMedium);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true)),
            Diagram(coordinateSystem(preserveAspectRatio=true)));
    end MoistAirSink;
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
          Ellipse(
            extent={{-76,58},{44,-62}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            lineThickness=1,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(points={{44,0},{78,0},{64,10}}),
          Line(points={{64,-10},{78,0}})}));
  end BoundaryConditions;

  package Connectors

    connector MoistAirInlet
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.Connectors.FluidInlet(redeclare package Medium = MoistAirMedium);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MoistAirInlet;

    connector MoistAirOutlet
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.Connectors.FluidOutlet(redeclare package Medium = MoistAirMedium);
    end MoistAirOutlet;
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
          Rectangle(
            extent={{-74,26},{-24,-24}},
            lineColor={28,108,200},
            lineThickness=1),
          Line(
            points={{-24,0},{24,0}},
            color={28,108,200},
            thickness=1),
          Rectangle(
            extent={{24,30},{82,-28}},
            lineColor={28,108,200},
            lineThickness=1,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}));
  end Connectors;

  package BaseClasses
    model MoistAirFlowModel
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.BaseClasses.FlowModel(P_in_0 = 0.9e5, P_out_0 = 0.9e5, h_in_0=1e3, h_out_0=1e3, Xi_0={0.1},
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
        redeclare package Medium = MoistAirMedium);

      import MetroscopeModelingLibrary.Units.Inputs;
      Inputs.InputPower W_input(start=0);
      Inputs.InputDifferentialPressure DP_input(start=0);
    equation
      W = W_input;
      DP = DP_input;
    end MoistAirFlowModel;

    model MoistAirIsoPFlowModel
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.BaseClasses.IsoPFlowModel(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
        redeclare package Medium = MoistAirMedium);

      import MetroscopeModelingLibrary.Units.Inputs;
      Inputs.InputPower W_input(start=0);
    equation
      W = W_input;
    end MoistAirIsoPFlowModel;

    model MoistAirIsoHFlowModel
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.BaseClasses.IsoHFlowModel(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
        redeclare package Medium = MoistAirMedium);

      import MetroscopeModelingLibrary.Units.Inputs;
      Inputs.InputDifferentialPressure DP_input(start=0);
    equation
      DP = DP_input;
    end MoistAirIsoHFlowModel;

    model MoistAirIsoPHFlowModel
      package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
      extends Partial.BaseClasses.IsoPHFlowModel(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
        redeclare package Medium = MoistAirMedium);
    end MoistAirIsoPHFlowModel;
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
       Rectangle(
            extent={{-46,49},{46,-47}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=1),
          Rectangle(
            extent={{26,18},{60,-16}},
            lineColor={28,108,200},
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-64,19},{-28,-17}},
            lineColor={28,108,200},
            lineThickness=1,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}));
  end BaseClasses;
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
        Ellipse(
          lineColor={102,102,102},
          fillColor={204,204,204},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Sphere,
          extent={{-60,-60},{60,60}})}));
end MoistAir;
