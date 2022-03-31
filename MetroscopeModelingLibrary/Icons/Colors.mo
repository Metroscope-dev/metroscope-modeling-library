within MetroscopeModelingLibrary.Icons;
partial record Colors
  // Icon parameters
  parameter String medium_name = "";

  // Component line colors
  parameter Color default_line_color = water_steam_line_color;
  parameter Color partial_line_color = {95,95,95};
  parameter Color water_steam_line_color = {28,108,200};
  parameter Color moist_air_line_color = {85,170,255};

  // Component fill colors
  parameter Color default_fill_color = water_steam_fill_color;
  parameter Color partial_fill_color = {230,230,230};
  parameter Color water_steam_fill_color = {255,255,255};
  parameter Color moist_air_fill_color = {255,255,255};

  // Inlet connector fill colors
  parameter Color default_inlet_fill_color = water_steam_fill_color;
  parameter Color partial_inlet_fill_color = {230,230,230};
  parameter Color water_steam_inlet_fill_color = water_steam_line_color;
  parameter Color moist_air_inlet_fill_color = moist_air_line_color;

  parameter Color line_color = DynamicSelect(default_line_color, if medium_name == "Partial" then partial_line_color
                                                       elseif medium_name == "WaterSteam" then water_steam_line_color
                                                       elseif medium_name == "MoistAir" then moist_air_line_color
                                                       else {0,0,0});
  parameter Color fill_color = DynamicSelect(default_fill_color,
                                             if medium_name == "Partial" then partial_fill_color
                                             elseif medium_name == "WaterSteam" then water_steam_fill_color
                                             elseif medium_name == "MoistAir" then moist_air_fill_color
                                             else {0,0,0});
  parameter Color inlet_fill_color = DynamicSelect(default_inlet_fill_color, if medium_name == "Partial" then partial_inlet_fill_color
                                                       elseif medium_name == "WaterSteam" then water_steam_inlet_fill_color
                                                       elseif medium_name == "MoistAir" then moist_air_inlet_fill_color
                                                       else {0,0,0});
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Colors;
