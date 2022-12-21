# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] <!--Make sure to add a link to the PR and issues related to your change-->

### Added <!--Make sure to add a link to the PR and issues related to your change-->
- Added [PR#325](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/325) created Slide Valve and added a Flue Gases control valve.
- Added [PR#319](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/319) densities and volumetric flow rates in the `FlowModel`

### Fixed <!--Make sure to add a link to the PR and issues related to your change-->
- Fixed [PR#318](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/318) `MoistAir_to_FlueGases` to have equal T between inlet and oulet.

### Changed <!--Make sure to add a link to the PR and issues related to your change-->
- Changed [PR#327](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/327) `fuelHeater` pipes location, Cp calculation, HX_configuration. In order to harmonise it with other other HX. And to have converging unit test. 

### Removed <!--Make sure to add a link to the PR and issues related to your change-->


## MML-v3.1 <!--Make sure to add a link to the PR and issues related to your change-->

### Added <!--Make sure to add a link to the PR and issues related to your change-->

- Added internal leaks in DryReheater, Reheater and SuperHeater [PR#306](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/306)
- Added units in leaks [PR#314](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/314)
- Added an air cooled condenser for CCGT modeling in [PR#305](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/305)
- Added a new configuration 'monophasic_counter_current' to NTUHeatExchange model. Added the possibility not to predefine the side of 'QCpMAX'. Added a parameter 'mixed_fluid' to specify if it's the 'hot' or the 'cold' fluid [PR#304](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/304)
- Added Metroscopia CCGT faulty model in [PR#301](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/301)

### Fixed
- Fixed [PR#311](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/311) `FlueGases.Machines.AirCompressor` Corrected the isentropic state by giving the correct composition

### Changed <!--Make sure to add a link to the PR and issues related to your change-->
- Modified `MultiFluid.Machines.CombustionChamber` [PR#310](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/310) can calculate HHV and LHV from the composition, and can be modified by user too
- Added `monophasic_counter_current` configuration in `Power.HeatExchange.NTUHeatExchange` [PR#304](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/304) with the ability not to pre identify the `Cpmax` side
- Modified `Fuel.BoundaryConditions.Source` [PR#309](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/309) to convert molar fraction to mass fraction

### Removed <!--Make sure to add a link to the PR and issues related to your change-->
 - Removed `adiabatic_compression` parameter in `Pump`, since it was not used [288](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/288) : [PR #289](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/289)

## MML-v3.0-beta <!--Make sure to add a link to the PR and issues related to your change-->

### Fixed <!--Make sure to add a link to the PR and issues related to your change-->
 - Fixed [#282](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/282) with [#287](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/287), by setting min and max in Positive/Negative MassFlowRate to 0, as it was an unnecessary protection.
 - Fixed [#273](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/273), steam extraction splitter'`x` are lower than 1 [PR#275](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/275)
 - Fixed `IsoHFlowModel` and `IsoPHFlowModel` now use `h_0` as `h` start value with [PR #265]([url](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/265))
 - Fixed `NTU HX` test configuration name for `shell_and_tubes` test, [PR #239](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/239)
 - Fixed superheater and multifluid case issues [#230](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/230) in [PR#231](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/231)
- Fixed multifluid submodels storage [#229](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/229) in [PR#231](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/231)
- Fixed Condenser parameter values [PR#221](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/221)
- Fixed TurbineLine examples in [PR#218](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/218)
- Fixed [#214](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/214): `.mat` files are ignored by default, but can still be added manually, [PR#215](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/215)

### Changed <!--Make sure to add a link to the PR and issues related to your change-->
- Moved `Partial/BaseClasses/IsoPHSimplifiedFlowModel.mo` to `Partial/Sensors/BaseSensor.mo` [PR #245](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/245)
- Replaced `PartialTransportModel` by `FlowModel` and removed `Xi_in`, `Xi_out`, `Q_in`, `Q_out`, `Qv_in`, `Qv_out`, `Qvm`, `rho_in`, `rho_out`, `rhom`, `DM`. Added `rho` instead of `rhom`, [PR#233](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/233)
- Ordered nuclear examples into subpackages called with power plant system names, [PR#219](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/219)

### Added <!--Make sure to add a link to the PR and issues related to your change-->
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

### Fixed

### Changed

### Added
Everything present in the lib at this time 😇

### Removed
Everything from MML2

## [2.4] 2022-01-04
