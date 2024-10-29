within MetroscopeModelingLibrary.DynamicComponents.Correlations;
function removeSensorSuffix
  input String name;
  output String newName;

algorithm
    newName := if Modelica.Utilities.Strings.find(name, "_sensor") > 0 then
    Modelica.Utilities.Strings.replace(name, "_sensor", "")
  else
    name;

end removeSensorSuffix;
