within ;
package MetroscopeModelingLibrary
  extends Modelica.Icons.Package;
annotation (uses(Modelica(version="4.0.0")),
  experiment(
    __Dymola_NumberOfIntervals=10,
    __Dymola_fixedstepsize=0.1,
    __Dymola_Algorithm="Euler"),
  __Dymola_experimentFlags(
    Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
    Evaluate=false,
    OutputCPUtime=false,
    OutputFlatModelica=false,
    LogStartValuesForIterationVariables=true));
end MetroscopeModelingLibrary;
