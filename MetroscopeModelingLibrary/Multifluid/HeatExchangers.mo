within MetroscopeModelingLibrary.MultiFluid;
package HeatExchangers
  extends MetroscopeModelingLibrary.Icons.HeatExchangePackage;

  model Economiser
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "cold",T_cold_in_0=76 + 273.15,P_cold_in_0 = 18 *1e5,Q_cold_0=178);

  end Economiser;

  model Superheater
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "hot",T_cold_in_0=140 + 273.15,P_cold_in_0 = 3.5 *1e5, Q_cold_0= 11);
    import MetroscopeModelingLibrary.Units.Inputs;


  end Superheater;
end HeatExchangers;
