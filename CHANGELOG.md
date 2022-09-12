# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed <!--Make sure to add a link to the PR and issues related to your change-->
 - Fixed [#99](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/99) in [#278](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/278), by adding a `not_used` effort variable to power inlets and outlets. Local balance is ensured by defining `C_in(not_used = 0)` in all power inlets. ⚠️ This fix works only because we have not several power inlets in one power connection set.
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
 - Added`HXmoistAirWater` component and reverse unit test, [PR #250](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/250) 
 - Added flue gases enthalpy start value to `hrsg_monophasic_HX`, [PR #264](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/264)
 - Added `mass_flow_rate_bias` fault in `BaseSensor`, to be able to declare faulty `FlowSensor` [PR #245](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/245)
 - Added `Mlb/h` unit in flow sensor [PR #240](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/240)
 - Added LP turbine with nozzle and condenser example [PR#224](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/224)
 - Added propagation of start values for water steam components, [PR#220](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/220)
 - Added Example/ReheaterLine, direct and reverse, [PR#216](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/216)

### Removed <!--Make sure to add a link to the PR and issues related to your change-->


## MML3-DTG-V2

### Fixed

### Changed

### Added
Everything present in the lib at this time 😇

### Removed
Everything from MML2

## [2.4] 2022-01-04
