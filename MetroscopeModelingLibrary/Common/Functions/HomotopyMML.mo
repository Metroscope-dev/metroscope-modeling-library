within MetroscopeModelingLibrary.Common.Functions;
function HomotopyMML
  extends Modelica.Icons.Function;
  input Real actual_value;
  input Real simplified_value;
  input Boolean use_homotopy = false;
  output Real value;
algorithm
  value := homotopy(actual_value, simplified_value); //if use_homotopy then homotopy(actual_value, simplified_value) else actual_value;
end HomotopyMML;
