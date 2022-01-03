within MetroscopeModelingLibrary.FlueGases.Machines;
model CombustionChamber
  extends CombustionChamberPartial;

  connector InputSpecificEnergy = input Modelica.Units.SI.SpecificEnergy;

  InputSpecificEnergy LHV;
equation
  LHV_int = LHV;
end CombustionChamber;
