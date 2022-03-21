# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
 - 📏 Fixed [#39](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/39) defined ./Units package, with all units defined in MML. 

### Changed

### Added
 - Added ./Connectors/InputConnectors to consistently declare all inputs
 - Added ./Constants package, to protect constants values and use them easily in the library
 - Added ./PartialTransport package, in which are implemented PartialTransportX.mo, where X is among h, P, Q, Xi, quantities that flow through the connector.
  And gathered all this PartialTransportX in one PartialTransportModel
 - Added initialization parameters (Q_in_0, P_out_0, etc.)

### Removed
 - 🧹 Removed everything from previous MML

## [2.4] 2022-01-04
