within MetroscopeModelingLibrary.Sensors_Control.RefMoistAir;
model RelativeHumiditySensor
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  extends MetroscopeModelingLibrary.Partial.Sensors_Control.BaseSensor(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RelativeHumidityIcon;

  // Relative Humidity
  parameter Real relative_humidity_0 = 0.5;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
  Real relative_humidity_pc(start=relative_humidity_0*100, min=0, max=100);
  Real pds;
  parameter Real H_start = 50 "Write here the build value of the quantity. This value will be used in the simulation.";


  // Display
  parameter String display_unit = "%" "Specify the display unit" annotation(choices(choice="", choice="%"));
  outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
  parameter String signal_unit = "%" annotation (choices(choice="1", choice="%"));

  Utilities.Interfaces.GenericReal      H_sensor(start=H_start) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,100}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,100})));
protected
  parameter Real k_mair = RefMoistAirMedium.k_mair;

equation
  pds = RefMoistAirMedium.Utilities.pds_pT(P, flow_model.T_in);
  flow_model.Xi = {relative_humidity*k_mair/(P/pds - relative_humidity)};
  relative_humidity_pc = relative_humidity*100;

  if signal_unit == "" then
    H_sensor = relative_humidity;
  elseif signal_unit == "%" then
    H_sensor = relative_humidity_pc;
  end if;

  annotation (Icon(graphics={Text(
        extent={{-100,-160},{102,-200}},
        textColor={0,0,0},
        textString=if display_output then
                   DynamicSelect("",String(relative_humidity_pc)+" %%")
                   else "")}));

end RelativeHumiditySensor;
