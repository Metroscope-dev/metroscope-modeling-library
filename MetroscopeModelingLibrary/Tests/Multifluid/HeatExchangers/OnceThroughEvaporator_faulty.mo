within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model OnceThroughEvaporator_faulty "Isolated test with thermal fouling"
  extends OnceThroughEvaporator_direct(evaporator(faulty=true));

equation
  evaporator.fouling = 20;

  annotation (experiment(StopTime=1, Tolerance=1e-6),
    Documentation(info="<html><p>Uses the same boundaries and clean coefficients as OnceThroughEvaporator_direct. The constant fouling reduces Kth by 20 percent, so the water absorbs less heat and the flue gases leave warmer.</p></html>"));
end OnceThroughEvaporator_faulty;
