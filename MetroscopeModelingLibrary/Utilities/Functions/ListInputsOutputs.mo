within MetroscopeModelingLibrary.Utilities.Functions;
function ListInputsOutputs

input String modelPath "Model path";
  output String realInputNames "Extracted RealInput variable names with label";
  output String realOutputNames "Extracted RealOutput variable names with label";
protected
  //String filePath;
  String[:] fileContent;
  String line, remaining, name;
  Integer posInput, posOutput, startIndex, nextIndex;
  String inputsList =  "";
  String outputsList =  "";
algorithm
  // Read the file
  // filePath := Modelica.Utilities.Files.fullPathName(modelPath);
  fileContent := Modelica.Utilities.Streams.readFile(modelPath);

  // Loop through each line
  for i in 1:size(fileContent, 1) loop
    line := fileContent[i];

    // Check for RealInput
    posInput := Modelica.Utilities.Strings.find(line, "RealInput");
    if posInput > 0 then
      startIndex := posInput + 9;
      remaining := Modelica.Utilities.Strings.substring(line, startIndex + 1, Modelica.Utilities.Strings.length(line));
      (name, nextIndex) := Modelica.Utilities.Strings.scanIdentifier(remaining, 1);
      inputsList := inputsList + name + "\n";
    end if;

    // Check for RealOutput
    posOutput := Modelica.Utilities.Strings.find(line, "RealOutput");
    if posOutput > 0 then
      startIndex := posOutput + 10;
      remaining := Modelica.Utilities.Strings.substring(line, startIndex + 1, Modelica.Utilities.Strings.length(line));
      (name, nextIndex) := Modelica.Utilities.Strings.scanIdentifier(remaining, 1);
      outputsList := outputsList + name + "\n";
    end if;
  end for;

  // Add headers before returning
  realInputNames := "Model Inputs:\n" + inputsList;
  realOutputNames := "Model Outputs:\n" + outputsList;


end ListInputsOutputs;
