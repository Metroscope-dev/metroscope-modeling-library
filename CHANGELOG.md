# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### To be addressed
### Fixed
 - 📏 Fixed [#39](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/39) defined ./Units package, with all units defined in MML. ([PR#68](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/68))

### Changed

### Added
 - Added Issue and PR Template ([PR#68](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/68))
 - Added Units ([PR#68](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/68))
 - Added ./Connectors/InputConnectors to consistently declare all inputs ([PR#70](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/70))
 - Added ./Constants package, to protect constants values and use them easily in the library ([PR#69](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/69))
 - Added ./PartialTransport package, in which are implemented PartialTransportX.mo, where X is among h, P, Q, Xi, quantities that flow through the component. Redisgned this into only one PartialTransportModel model ([PR#73](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/73)), fixing [#73](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/73)
  And gathered all this PartialTransportX in one PartialTransportModel ([PR#70](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/70))
 - Added initialization parameters (Q_in_0, P_out_0, etc.) ([PR#70](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/70))
 - Added Base classes and checked locally balances ([PR#70](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/70))
 - Added Sources and Sink ([PR#71](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/71))
 - Added WaterSteamMedium and WaterBaseClasses ([PR#72](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/72))
 - Added first Partial sensors and inherited them into Sensors/WaterSteam ([PR#73](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/73))
 - Added WaterSteam/BaseClasses/Tests ([PR#76](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/76))
 - Added Moist air package, inherited classes and added tests ([PR#78](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/78))
 - Added Pipes package, inherited classes and added tests ([PR#83](https://github.com/Metroscope-dev/metroscope-modeling-library/pull/83))

### Removed
 - Removed PartialBoundaryCondition model, and separated in two : source and sink
 - 🧹 Removed everything from previous MML

## [2.4] 2022-01-04
