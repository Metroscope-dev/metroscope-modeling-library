within MetroscopeModelingLibrary.FlueGases.BoundaryConditions;
model Source
  package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Common.BoundaryConditions.Source(redeclare
      package                                                                        Medium =
        FlueGasesMedium);
  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}));
end Source;
