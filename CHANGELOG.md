# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed <!--Make sure to add a link to the PR and issues related to your change-->
 - Fixed superheater and multifluid case issues [#230](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/230) in [PR#231](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/231)
 - Fixed multifluid submodels storage [#229](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/229) in [PR#231](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/231)
 - Fixed Condenser parameter values [PR#221](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/221)
 - Fixed TurbineLine examples in [PR#218](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/218)
 - Fixed [#214](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/214): `.mat` files are ignored by default, but can still be added manually, [PR#215](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/215)

### Changed <!--Make sure to add a link to the PR and issues related to your change-->
 - Replaced `PartialTransportModel` by `FlowModel` and removed `Xi_in`, `Xi_out`, `Q_in`, `Q_out`, `Qv_in`, `Qv_out`, `Qvm`, `rho_in`, `rho_out`, `rhom`, `DM`. Added `rho` instead of `rhom`, [PR#233](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/233)
 - Ordered nuclear examples into subpackages called with power plant system names, [PR#219](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/219)

### Added <!--Make sure to add a link to the PR and issues related to your change-->
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
