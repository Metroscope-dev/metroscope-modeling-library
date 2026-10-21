within MetroscopeModelingLibrary;
package SensorsControl

  package WaterSteam

    model TemperatureSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

        extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;
    end PressureSensor;

    model DeltaPressureSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;
    end FlowSensor;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            lineColor={102,102,102},
            fillColor={28,108,200},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{-60,-60},{60,60}})}));
  end WaterSteam;

  package FlueGases

    model TemperatureSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    end PressureSensor;

    model DeltaPressureSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

    end FlowSensor;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            lineColor={102,102,102},
            fillColor={204,204,204},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Sphere,
            extent={{-60,-60},{60,60}}),
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            lineColor={102,102,102},
            fillColor={204,204,204},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Sphere,
            extent={{-60,-60},{60,60}}),
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            lineColor={102,102,102},
            fillColor={95,95,95},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{-60,-60},{60,60}})}));
  end FlueGases;

  package Fuel

    model TemperatureSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    end PressureSensor;

    model DeltaPressureSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

    end FlowSensor;

    model Chromatograph
      Utilities.Interfaces.GenericReal X_CO2 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={20,40}), iconTransformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={84,36})));
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors.BaseSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;

        // Atomic mass
      constant Real amC=12.01115 "Carbon atomic mass";
      constant Real amH=1.00797 "Hydrogen atomic mass";
      constant Real amO=15.9994 "Oxygen atomic mass";
      constant Real amN=14.0067 "Nitrogen atomic mass";

      // Molar mass of species of interest
      Real amCH4 "CH4 molecular mass";
      Real amC2H6 "C2H6 molecular mass";
      Real amC3H8 "C3H8 molecular mass";
      Real amC4H10 "C4H10 molecular mass";
      Real amCO2 "CO2 molecular mass";
      Real amN2 "H2O molecular mass";

      // Mole fractions
      Real X_molar_CH4(start=0.92);
      Real X_molar_C2H6(start=0.048);
      Real X_molar_C3H8(start=0.005);
      Real X_molar_C4H10_n_butane(start=0.002);
      Real X_molar_N2(start=0.015);
      Real X_molar_CO2(start=0.01);

      // Mean molecular mass
      Real mean_molecular_mass(start=17);

      Utilities.Interfaces.GenericReal X_C2H6 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,40}), iconTransformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={-60,76})));
      Utilities.Interfaces.GenericReal X_C3H8 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-40,40}), iconTransformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={-22,104})));
      Utilities.Interfaces.GenericReal X_C4H10_n_butane annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-20,40}), iconTransformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={20,104})));
      Utilities.Interfaces.GenericReal X_N2                           annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,40}), iconTransformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={60,78})));
    equation

      // Molar mass of species of interest
    public
      Utilities.Interfaces.GenericReal X_CH4 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-80,40}), iconTransformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={-84,38})));
    equation
      amCH4 = amC + 4*amH;
      amC2H6 = 2*amC + 6*amH;
      amC3H8 = 3*amC + 8*amH;
      amC4H10 = 4*amC + 10*amH;
      amN2 = 2*amN;
      amCO2 = amC + 2*amO;

      // Composition mass fraction
      X_CH4/100 = Xi[1]; // methane
      X_C2H6/100 = Xi[2]; // ethane
      X_C3H8/100 = Xi[3]; // propane
      X_C4H10_n_butane/100 = Xi[4]; // butane
      X_N2/100 = Xi[5]; // nitrogen
      X_CO2/100 = Xi[6]; // carbon dioxyde

      // Mean Molecular Mass: this gives the correct results only if the molar fraction is given as an input, if the mass fraction is given, this quantity is useless
      mean_molecular_mass = X_molar_CH4*amCH4 + X_molar_C2H6*amC2H6 + X_molar_C3H8*amC3H8 + X_molar_C4H10_n_butane*amC4H10 + X_molar_N2*amN2 + X_molar_CO2*amCO2;

      // Mass and mole fraction relation
      X_molar_CH4 = X_CH4/amCH4 * mean_molecular_mass;
      X_molar_C2H6 = X_C2H6/amC2H6 * mean_molecular_mass;
      X_molar_C3H8 = X_C3H8/amC3H8 * mean_molecular_mass;
      X_molar_C4H10_n_butane = X_C4H10_n_butane/amC4H10 * mean_molecular_mass;
      X_molar_N2 = X_N2/amN2 * mean_molecular_mass;
      X_molar_CO2 = X_CO2/amCO2 * mean_molecular_mass;

      annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
                       graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="X")}),    Diagram(coordinateSystem(extent={{-100,-100},{100,
                100}})));
    end Chromatograph;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            lineColor={102,102,102},
            fillColor={204,204,204},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Sphere,
            extent={{-60,-60},{60,60}}),
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            lineColor={102,102,102},
            fillColor={204,204,204},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Sphere,
            extent={{-60,-60},{60,60}}),
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            lineColor={102,102,102},
            fillColor={213,213,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{-60,-60},{60,60}})}));
  end Fuel;

  package MoistAir

    model TemperatureSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    end PressureSensor;

    model DeltaPressureSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

    end FlowSensor;

    model RelativeHumiditySensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends MetroscopeModelingLibrary.Partial.Sensors_Control.BaseSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RelativeHumidityIcon;

      // Relative Humidity
      parameter Real relative_humidity_0 = 0.5 annotation(Dialog(tab="Initialization", group="Start values"));
      Real relative_humidity(start=relative_humidity_0, min=0, max=1);
      Real relative_humidity_pc(start=relative_humidity_0*100, min=0, max=100);

      // Sensor signal parameters
      parameter Real H_start = 50 "Write here the build value of the quantity. This value will be used in the simulation." annotation(Dialog(tab="General", group="Sensor signal parameters"));
      parameter String display_unit = "%" "Specify the display unit" annotation(choices(choice="", choice="%"),
      Dialog(tab="General", group="Sensor signal parameters"));
      parameter String signal_unit = "%" "Specify the signal unit. This should be the unit of H_start and of the tag linked to the sensor." annotation (choices(choice="1", choice="%"),
      Dialog(tab="General", group="Sensor signal parameters"));


      Utilities.Interfaces.GenericReal      H_sensor(start=H_start) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,100}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,100})));
    equation
      flow_model.Xi[1] = MoistAirMedium.massFraction_pTphi(P, flow_model.T_in, relative_humidity);
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
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid,
            extent={{-60,-60},{60,60}},
            pattern=LinePattern.None)}));
  end MoistAir;

  package Power
    model PowerSensor

      import MetroscopeModelingLibrary.Utilities.Units.Inputs;

      Utilities.Units.Power W;
                      // Power in W
      Real W_MW(min=0, nominal=100); // Power in MW

      // Causality display parameters
      parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
        annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"),
        Dialog(tab="General", group="Causality display parameters"));
      parameter String causality = "" "Specify which parameter is calibrated by this sensor" annotation(Dialog(tab="General", group="Causality display parameters"));
      parameter Boolean show_causality = true "Used to show or not the causality";

      // Sensor signal parameters
      parameter Real W_start = 100 "Write here the build value of the quantity. This value will be used in the simulation." annotation(Dialog(tab="General", group="Sensor signal parameters"));
      parameter String signal_unit = "MW" "Specify the signal unit. This should be the unit of W_start and of the tag linked to the sensor." annotation(choices(choice="MW", choice="W"),
      Dialog(tab="General", group="Sensor signal parameters"));
      parameter String display_unit = "MW" "Specify the display unit"
        annotation(choices(choice="MW", choice="W"),
        Dialog(tab="General", group="Sensor signal parameters"));

      parameter Boolean display_output = false "Used to switch ON or OFF output display";

      MetroscopeModelingLibrary.Power.Connectors.Inlet C_in annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
      MetroscopeModelingLibrary.Power.Connectors.Outlet C_out annotation (Placement(transformation(extent={{88,-10},{108,10}}), iconTransformation(extent={{88,-10},{108,10}})));
      Utilities.Interfaces.GenericReal      W_sensor(start=W_start) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,100}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,100})));
    equation
      // Conservation of power
      C_in.W + C_out.W = 0; // C_out.W < 0 if power flows out of component, as for mass flows

      // Measure
      W = C_in.W;
      W_MW = W/1e6;

      if signal_unit == "MW" then
        W_sensor = W_MW;
      else
        W_sensor = W;
      end if;

      annotation (Icon(graphics={Text(
          extent={{-100,-160},{102,-200}},
          textColor={0,0,0},
          textString=if display_output then
                     if display_unit == "W" then DynamicSelect("",String(W)+" W")
                     else DynamicSelect("",String(W_MW)+" MW")
                     else ""),
         Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor=if sensor_function == "BC" then {238, 46, 47} elseif sensor_function == "Calibration" then {107, 175, 17} else {255, 255, 255},
            fillPattern=if sensor_function == "BC" or sensor_function == "Calibration" then FillPattern.Solid else FillPattern.None),
          Text(
            extent={{-100,-120},{100,-160}},
            textColor={107,175,17},
            textString=if show_causality then "%causality" else ""),
          Line(
            points={{100,-60},{140,-60},{140,-140},{100,-140}},
            color={107,175,17},
            arrow=if causality == "" or show_causality == false then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
            thickness=0.5,
            pattern=if causality == "" or show_causality == false then LinePattern.None else LinePattern.Solid,
            smooth=Smooth.Bezier),
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,80},{80,-80}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Text(
            extent={{-60,60},{60,-60}},
            textColor={0,0,0},
            textString="W")}));
    end PowerSensor;
    annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          lineColor={102,102,102},
          fillColor={244,125,35},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
  end Power;

  package Outline

    model OpeningSensor

      import MetroscopeModelingLibrary.Utilities.Units.Inputs;

      parameter Utilities.Units.Percentage Opening_pc_0 = 15 annotation(Dialog(tab="Initialization", group="Start values"));
      Inputs.InputPercentage Opening_pc(unit="1", start=Opening_pc_0, min=0, max=100, nominal=Opening_pc_0); // Opening in percentage

      // Causality display parameters
      parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
        annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"),
        Dialog(tab="General", group="Causality display parameters"));
      parameter String causality = "" "Specify which parameter is calibrated by this sensor" annotation(Dialog(tab="General", group="Causality display parameters"));
      parameter Boolean display_output = true "Used to switch ON or OFF output display";

      // Sensor signal parameters
      parameter Real Opening_start = 15 "Write here the build value of the quantity. This value will be used in the simulation." annotation(Dialog(tab="General", group="Sensor signal parameters"));
      parameter String signal_unit = "%" "Specify the signal unit. This should be the unit of Opening_start and of the tag linked to the sensor." annotation(choices(choice="%" "percentage, between 0 and 100", choice="" "No unit, between 0 and 1"),
      Dialog(tab="General", group="Sensor signal parameters"));

      Modelica.Blocks.Interfaces.RealOutput Opening(unit="1", min=0, max=1, nominal=Opening_pc_0/100, start=Opening_pc_0/100)
        annotation (Placement(transformation(
            extent={{-27,-27},{27,27}},
            rotation=270,
            origin={0,-20}), iconTransformation(extent={{-27,-27},{27,27}},
            rotation=270,
            origin={0,-102})));
      Utilities.Interfaces.GenericReal opening_sensor(start=Opening_start) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,102}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,102})));
    equation
      Opening_pc = Opening * 100;

      if signal_unit == "%" then
        opening_sensor = Opening_pc;
      else
        opening_sensor = Opening;
      end if;
      annotation (Icon(graphics={ Text(
              extent={{-100,200},{100,160}},
              textColor={0,0,0},
              textString=if display_output then DynamicSelect("",String(Opening_pc)+" %%")
              else ""),
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor=if sensor_function == "BC" then {238, 46, 47} elseif sensor_function == "Calibration" then {107, 175, 17} else {255, 255, 255},
            fillPattern=if sensor_function == "BC" or sensor_function == "Calibration" then FillPattern.Solid else FillPattern.None),
          Text(
            extent={{-100,-120},{100,-160}},
            textColor={107,175,17},
            textString=if causality <> "" then "%causality" else ""),
          Line(
            points={{100,-60},{140,-60},{140,-140},{100,-140}},
            color={107,175,17},
            arrow=if causality == "" then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
            thickness=0.5,
            pattern=if causality == "" then LinePattern.None else LinePattern.Solid,
            smooth=Smooth.Bezier),
          Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=0.5),
          Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="O")}));
    end OpeningSensor;

    model VRotSensor
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.VRotIcon;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;

      Modelica.Blocks.Interfaces.RealOutput VRot(min=0, nominal=2000, start=2000) "rotations per minute" annotation (Placement(transformation(
            extent={{-27,-27},{27,27}},
            rotation=270,
            origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
            rotation=270,
            origin={0,-102})));
    end VRotSensor;
  annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            extent={{-70,70},{70,-70}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),
        Polygon(
          points={{-20,-57},{0,-91},{20,-57},{-20,-57}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
  end Outline;

  package RefMoistAir

    model TemperatureSensor
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    end PressureSensor;

    model DeltaPressureSensor
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

    end FlowSensor;

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
      parameter Boolean display_output = false "Used to switch ON or OFF output display";
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
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            fillColor={0,127,127},
            fillPattern=FillPattern.Solid,
            extent={{-60,-60},{60,60}},
            pattern=LinePattern.None)}));
  end RefMoistAir;
annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          fillColor={245,245,245},
          fillPattern=FillPattern.Solid,
          extent={{-70,-70},{70,70}}),
        Line(points={{0,70},{0,40}}),
        Line(points={{22.9,32.8},{40.2,57.3}}),
        Line(points={{-22.9,32.8},{-40.2,57.3}}),
        Line(points={{37.6,13.7},{65.8,23.9}}),
        Line(points={{-37.6,13.7},{-65.8,23.9}}),
        Ellipse(
          lineColor={64,64,64},
          fillColor={255,255,255},
          extent={{-12,-12},{12,12}}),
        Polygon(
          rotation=-17.5,
          fillColor={64,64,64},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-5.0,0.0},{-2.0,60.0},{0.0,65.0},{2.0,60.0},{5.0,0.0}}),
        Ellipse(
          fillColor={64,64,64},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-7,-7},{7,7}})}),
           Documentation(info="<html>
  <p>Licensed by Metroscope under the Modelica License 2 </p>
<p>Copyright © 2023, Metroscope.</p>
<p>This Modelica package is free software and the use is completely at your own risk; it can be redistributed and/or modified under the terms of the Modelica License 2. </p>
</html>"));
end SensorsControl;
