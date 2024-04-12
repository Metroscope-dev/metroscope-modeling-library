# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
Use [gitmoji](https://gitmoji.dev/) to identify your changes.

## [Unreleased]

### ✨ Added <!--Make sure to add a link to the PR and issues related to your change-->
- New feature to display simulation outputs on model diagrams [#482](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/482)

### 🐛 Fixed <!--Make sure to add a link to the PR and issues related to your change-->

### 💥 Changed <!--Make sure to add a link to the PR and issues related to your change-->

### 🔥 Removed <!--Make sure to add a link to the PR and issues related to your change-->


## MML - v3.8.0

### ✨ Added <!--Make sure to add a link to the PR and issues related to your change-->
- Added sensors name in sensors icons [#473](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/473)
- Added units for volume, surface tension, dynamic viscosity and thermal conductivity [#474](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/474)
- Added input connectors for all units [#475](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/475)
- Added sensor background color feature in sensors icons [#477](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/477)
- Added Fogging component [PR#460](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/460)
- Added a feature to show the causality in sensors icon [PR#479](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/479)

### 🐛 Fixed <!--Make sure to add a link to the PR and issues related to your change-->
- Fix flow direction in valve [#468](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/468)


## MML - v3.7.0

### ✨ Added <!--Make sure to add a link to the PR and issues related to your change-->
- Added compressor and gas turbine internal faults, condenser `Qv_cold_in` decrease fault, and missing Fuel and FlueGases pipes, with all corresponding tests [PR#457](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/457)

### 💥 Changed <!--Make sure to add a link to the PR and issues related to your change-->
- Pressure losses equations in Pipe, ControlValve and SlidingValve now use inlet density instead of mean density. This change can affect calibrated models. [PR#450](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/450)


## MML - v3.6.1

### ✨ Added <!--Make sure to add a link to the PR and issues related to your change-->

- 🆕 Added fuel valve component [PR#446](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/446)
- ✨ Add and correct initialization parameters in flue gases [PR#436](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/436)
- ✨ Add indicators in the HX models [PR#443](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/443)
- ✨ Added a warning if the steam admitted in the condenser is superheated [PR#445](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/445)
- 🆕 Tank component [PR#434](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/434)

### 🐛 Fixed <!--Make sure to add a link to the PR and issues related to your change-->

- 🔧 Initialisation parameters and unit tests of the multifluid heat exchangers fix for a better convergence [PR#439](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/439)
- 🔧 Added `rho_0` as an initialization parameter in the `FlowModel` [PR#440](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/440)

### 💥 Changed <!--Make sure to add a link to the PR and issues related to your change-->

- 💅 Fixed all icons issues in [PR#434](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/434)

### 🔥 Removed <!--Make sure to add a link to the PR and issues related to your change-->

## MML - v3.6.0

### :sparkles: Added

- Added DT and DH variables in flow model to simplify access to these variables [PR#421](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/421) fixing [#397](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/397)
- Added `assert` statements in heat exchangers power components [PR#420](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/420), fixing [#417](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/417)

### :boom: Changed
- Steam generators now have power connectors for thermal power and embedded feedwater mass flow rate bias [PR#418](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/418)

## MML-v3.5.0

### :sparkles: Added
- SteamTurbineWithNozzle component to replace StodolaTurbine when there is a nozzle [PR#396](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/396)
- PressureCut component with the medium MoistAir [PR#398](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/398/commits)
- AirCooledCondenser_with_subcooling component adding subcooling to the ACC model [PR#394](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/394)

### :boom: Changed
- Renamed parameter `rhmin` of pumps in `rh_min`[PR#414](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/396)
- Renamed parameter `Cvmax` of pumps in `Cv_max`[PR#414](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/396)
- Deleted compressor power inlet in gas turbine model and renamed `Wmech` into `W_shaft` [PR#409](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/409)
- Renamed internal variables of HX : Reheater, DryReheater, SuperHeater, AirCooledCondenser_with_subcooling [PR#406](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/406)
- Renamed internal variables of SteamTurbine and SteamTurbineWithNozzle, might break start values models [PR#404](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/404)
- Renamed internal variables of Reheater and DryReheater, especially `separating_plate_leak` renamed in `partition_plate_leak` [PR#402](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/396)
- Renamed internal variables of Steam Extraction Splitter (might break model with start values) [PR#401](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/401)
- Stodola turbine has been renamed SteamTurbine and does not have a nozzle [PR#396](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/396)
- `kPa` and `MPa` units are splitted between gauge and absolute in pressure sensor [PR#391](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/391)

## MML-v3.4.1

### :boom: Changed
- Sensors now contain a flow model to ease convergence [PR#383](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/383)

### :bug: Fixed
- Pipes now can have a unique flow direction according to the pressure difference direction [PR#385](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/385)

## MML-v3.4.0

### :sparkles: Added

- Added a version number in root `package.mo`, to allow appropriate versioning in models, using `uses(MetroscopeModelingLibrary(version="3.1.0"))` annotation. [PR#364](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/364)
- Added `MPa` and `kPa` pressure units in pressure sensors [PR#362](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/362), fixing [#361](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/361)

### :boom: Changed

- Updated leak model so they use inheritance [PR#373](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/373)
- Abstract sensors connectors are renamed `C_in` and `C_out`, breaking change for topological models [PR#372](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/372)
- Changed default unused value for enthalpy in opposite flow direction to 1e6 [PR#374](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/374)
- [PR#368](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/368) the `fouling` fault in `Pipe` and `closed` fault in `SlideVale` are changed to percentage. The values given to those faults should be a percentage (%) and not a value less than 1 as before. Therefore, it is a breaking change for faulty models in prevous versions.
- [PR#377](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/377) the `DP` in the `CombustionChamber` model is replaced by a `Kfr` by adding a pipe. This is a breaking change for models using previous versions.

### :bug: Fixed
- [PR#376](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/376) fixed the outlet temperatures of the evaporator.

## MML-v3.3.1
### :sparkles: Added

- [PR#355](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/355) added `PressureCut` and `Leak` components for the flue gases medium.
- [PR#356](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/356) Added an  `valve_closed` in the `SlideValve` component to model slight decrease in the valve opening.

## MML-v3.3.0

### :bug: Fixed
- Fix start values of outlet flows in heater with internal leaks [PR#349](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/349)
- [PR#351](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/351) Correction of the sign of the nominal temperature drop in the `hrsg_monophasic_HX` and in the `FuelHeater`. This is a breaking change for the models calibrated before.

## MML-v3.2.2

### :sparkles: Added
- [PR#338](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/338) Added the calibration diagram of Metroscopia CCGT.
- Added water manifold in WaterSteam/Pipes [PR#341](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/341)

### :boom: Changed <!--Make sure to add a link to the PR and issues related to your change-->
- [PR#346](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/346) adds inlet and outlet connectors to abstract sensors to be able to put them in the line, to avoid mapping mistakes. Closes [issue#284](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/284)

### :bug: Fixed
- [PR#347](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/347) Modification of the pressure drop equation of a valve so it can simulate the conditions where the pressure at the outlet is higher than the pressure at the inlet. No impact on models.

## MML-v3.2.1 <!--Make sure to add a link to the PR and issues related to your change-->

### :sparkles: Added
- [PR#331](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/331) replace control valves by slide valves at the steam turbines admission in CCGT Metroscopia.
- Added [PR#330](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/330) `HeatLoss` component for the `MoistAir` medium. Answers [issue#329](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/329).

## MML-v3.2 <!--Make sure to add a link to the PR and issues related to your change-->

### :bug: Fixed 
- [PR#326](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/326) pipe order and assigned temperatures of the `hrsg_monophasic_HX`. Model calibrated with previous version of the library should be recalibrated. The parameter `nominal_hot_side_temperature_rise` was changed to `nominal_hot_side_temperature_drop` and needs to be updated in the models. The default configuration of the HRSG HX is now `monophasic_counter_current`. Answers issues [#268](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/268) and [#254](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/254).
- Fixed [PR#318](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/318) `MoistAir_to_FlueGases` to have equal T between inlet and oulet.

### :sparkles: Added <!--Make sure to add a link to the PR and issues related to your change-->
- Added [PR#325](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/325) created Slide Valve and added a Flue Gases control valve.
- Added [PR#319](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/319) densities and volumetric flow rates in the `FlowModel`

### :boom: Changed <!--Make sure to add a link to the PR and issues related to your change-->
- Changed [PR#327](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/327) `fuelHeater` pipes location, HX_configuration. In order to harmonise it with other other HX. And to have converging unit test. + Breaking retrocompatibility change : new Cp calculation implies to give 2 more parameters `nominal_cold_side_temperature_rise` and `nominal_hot_side_temperature_drop`


## MML-v3.1 <!--Make sure to add a link to the PR and issues related to your change-->

### :sparkles: Added <!--Make sure to add a link to the PR and issues related to your change-->

- Added internal leaks in DryReheater, Reheater and SuperHeater [PR#306](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/306)
- Added units in leaks [PR#314](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/314)
- Added an air cooled condenser for CCGT modeling in [PR#305](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/305)
- Added a new configuration 'monophasic_counter_current' to NTUHeatExchange model. Added the possibility not to predefine the side of 'QCpMAX'. Added a parameter 'mixed_fluid' to specify if it's the 'hot' or the 'cold' fluid [PR#304](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/304)
- Added Metroscopia CCGT faulty model in [PR#301](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/301)

### :bug: Fixed
- Fixed [PR#311](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/311) `FlueGases.Machines.AirCompressor` Corrected the isentropic state by giving the correct composition

### :boom: Changed <!--Make sure to add a link to the PR and issues related to your change-->
- Modified `MultiFluid.Machines.CombustionChamber` [PR#310](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/310) can calculate HHV and LHV from the composition, and can be modified by user too
- Added `monophasic_counter_current` configuration in `Power.HeatExchange.NTUHeatExchange` [PR#304](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/304) with the ability not to pre identify the `Cpmax` side
- Modified `Fuel.BoundaryConditions.Source` [PR#309](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/309) to convert molar fraction to mass fraction

### :fire: Removed <!--Make sure to add a link to the PR and issues related to your change-->
 - Removed `adiabatic_compression` parameter in `Pump`, since it was not used [288](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/288) : [PR #289](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/289)

## MML-v3.0-beta <!--Make sure to add a link to the PR and issues related to your change-->

### :bug: Fixed <!--Make sure to add a link to the PR and issues related to your change-->
 - Fixed issue [#191](https://github.com/Metroscope-dev/metroscope-modeling-library/issues) : now components cannot be put in the model with the wrong connections, thanks to assertions in base classes ([PR#274](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/274))
 - Fixed [#282](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/282) with [#287](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/287), by setting min and max in Positive/Negative MassFlowRate to 0, as it was an unnecessary protection.
 - Fixed [#273](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/273), steam extraction splitter'`x` are lower than 1 [PR#275](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/275)
 - Fixed `IsoHFlowModel` and `IsoPHFlowModel` now use `h_0` as `h` start value with [PR #265](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/265)
 - Fixed `NTU HX` test configuration name for `shell_and_tubes` test, [PR #239](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/239)
 - Fixed superheater and multifluid case issues [#230](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/230) in [PR#231](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/231)
- Fixed multifluid submodels storage [#229](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/229) in [PR#231](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/231)
- Fixed Condenser parameter values [PR#221](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/221)
- Fixed TurbineLine examples in [PR#218](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/218)
- Fixed [#214](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/214): `.mat` files are ignored by default, but can still be added manually, [PR#215](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/215)

### :boom: Changed <!--Make sure to add a link to the PR and issues related to your change-->
- Moved `Partial/BaseClasses/IsoPHSimplifiedFlowModel.mo` to `Partial/Sensors/BaseSensor.mo` [PR #245](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/245)
- Replaced `PartialTransportModel` by `FlowModel` and removed `Xi_in`, `Xi_out`, `Q_in`, `Q_out`, `Qv_in`, `Qv_out`, `Qvm`, `rho_in`, `rho_out`, `rhom`, `DM`. Added `rho` instead of `rhom`, [PR#233](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/233)
- Ordered nuclear examples into subpackages called with power plant system names, [PR#219](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/219)

### :sparkles: Added <!--Make sure to add a link to the PR and issues related to your change-->
 - Added `psiA` and `psiG` distinction in pressure sensor [PR#252](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/252)
 - Added `LMTDHeatExchange` function, `LMTDfuelHeater`component and reverse unit test. [PR #266](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/266)
 - Added`HXmoistAirWater` component and reverse unit test, [PR #250](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/250) 
 - Added flue gases enthalpy start value to `hrsg_monophasic_HX`, [PR #264](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/264)
 - Added `mass_flow_rate_bias` fault in `BaseSensor`, to be able to declare faulty `FlowSensor` [PR #245](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/245)
 - Added `Mlb/h` unit in flow sensor [PR #240](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/240)
 - Added LP turbine with nozzle and condenser example [PR#224](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/224)
 - Added propagation of start values for water steam components, [PR#220](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/220)
 - Added Example/ReheaterLine, direct and reverse, [PR#216](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/216)

## MML3-DTG-V2

### :sparkles: Added
- Everything present in the lib at this time 😇

### :fire: Removed
- Everything from MML2

## [2.4] 2022-01-04
