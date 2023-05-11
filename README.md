# Metroscope Modeling Library

## Content

The Metroscope Modeling Library is a set of modelica packages for industrial components such as pumps, turbines, heat exchangers, to build steady-state digital twins of industrial process. 

The library describes the components of a thermodynamic process at a system level, focusing on the interactions between inputs and outputs of a component. It is thus considered as a 0D modeling approach.

There is no detailed description of the physical phenomena, but a focus on the macro behaviour of the components. It is a thermalhydraulic description of the system, focusing on computing pressures, mass flow rates, enthalpies and temperatures throughout the system. All MML models are described at steady state : there are no time dependent equations.

MML was built based on the [ThermoSysPRo library](https://thermosyspro.com/), but it has some some key differences :

- **In MML3, flow is assumed to go only in one direction and cannot be reversed**
- For enthalpy propagation, stream connectors are used instead of the `h_vol` solution used in TSP (and in previous versions of MML, until MML2.4)
- All components are acausal, you can impose different sets of physical quantities or conversely set the component characteristics, for the same model.

## Usage

- When creating a model, you should only use official releases. Official releases can be downloaded from [here](https://github.com/Metroscope-dev/metroscope-modeling-library/releases).
- In your model, you should add a line in `annotation`, containing the following : `uses(MetroscopeModelingLibrary(version(="3.0.1"))` in which you replace the version number (`"3.0.1"`) by the one you use. That way you will get a warning if you use your model with a wrong version of the library.

## Contributions

Your contributions are very welcome. You may contribute by [reporting a bug](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/new?assignees=&labels=%F0%9F%90%9Bbug&projects=&template=bug_report.md&title=%5BBUG%5D), [asking for a new feature](https://github.com/Metroscope-dev/metroscope-modeling-library/issues/new?assignees=&labels=%E2%9C%A8enhancement&projects=&template=feature_request.md&title=%5BFEATURE%5D) to be implemented, or [propose some of your own changes](https://github.com/Metroscope-dev/metroscope-modeling-library/compare). Follow the contributing guide for further details.

## License

License file is [Metroscope Contributor License Agreement For MML](Metroscope_contributor_license_agreement_for_mml.docx)

## Attribution notices

Copyright 2021 Metroscope

Metroscope Modeling Library is a Derivative Work of ThermoSysPro:
- [ThermoSysPro home page](https://thermosyspro.com)
- [ThermoSysPro license](https://github.com/ThermoSysPro/ThermoSysPro/blob/master/LICENSE.md)
