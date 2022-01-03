within MetroscopeModelingLibrary.Common.Sensors;
model HumiditySensor
  extends BaseSensor;

  Real humidity_percent;  // Humidity as percent
  Real humidity;

equation

  humidity = X_in[1];
  humidity_percent = X_in[1]*100;


  annotation (Icon(graphics={Text(
          extent={{-108,44},{108,-50}},
          textColor={0,0,0},
          textString="X")}));
end HumiditySensor;
