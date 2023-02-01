# Metroscope Modeling Library

## Content

 The Metroscope Modeling Library contains modelica modules for industrial components such as pumps, turbines, heat exchangers, to build steady-state digital twins of industrial process. The associated documentation contains explanation of the library equations. 

## Usage

- When creating a Digital Twin (also called model or even `Parametrization` in license file), you should only use official releases of `Metroscope Modeling Library`. Official releases can be downloaded from [here](https://github.com/Metroscope-dev/metroscope-modeling-library/releases).
- In your model, you should add a line in `annotation`, containing the following : `uses(MetroscopeModelingLibrary(version(="3.0.1"))` in which you replace the version number (`"3.0.1"`) by the one you use. That way you will get a warning if you use your model with a wrong version of the library.

## Reporting a bug

- Report your issues on [issues page](https://github.com/Metroscope-dev/metroscope-modeling-library/issues)
- Do not attach sensitive data to the GitHub issue you create
- Provide following information
  - A clear and concise description of what the bug is
  - Steps to reproduce the bahavior
  - A clear and concise description of what you expected to happen.
  - Your configuration (OS, Modelica version, Metroscope Modeling Library version, modeling software & version)

## Contributions

Your contributions are very welcome. To contribute, you have to clone repository `Metroscope Modeling Library`: `git clone git@github.com:Metroscope-dev/metroscope-modeling-library.git`

To suggest changes, you have to:
- checkout main branch: `git checkout master`
- pull last changes: `git pull`
- create a new branch: `git checkout -b new-branch-name`
- commit your changes & push your code
- [create a pull request](https://github.com/Metroscope-dev/metroscope-modeling-library/pulls)
  - Give a meaningfule title to your pull request (examples: `BUG FIX - Stodola Turbine` or `NEW COMPONENT - <NEW_COMPONENT_NAME>`)
  - Provide a comment explaining the changes
    - Aim of the changes
    - Description of the changes
    - If associated to an open GitHub issue, provide its url

For the sake of efficiency, create a draft pull request if you still have work to do before having your pull request reviewed ([see Github documentation if needed](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/creating-a-pull-request)).

When your pull request is ready for review, change its stage ([see documentation if needed](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/changing-the-stage-of-a-pull-request)) and notify the `#physical-modeling` slack channel Your pull request will be reviewed; if validated, it will be merged on main branch by Metroscope.

## License

License file is [Metroscope Contributor License Agreement For MML](Metroscope_contributor_license_agreement_for_mml.docx)

## Attribution notices

Copyright 2021 Metroscope

Metroscope Modeling Library is a Derivative Work of ThermoSysPro:
- [ThermoSysPro home page](https://thermosyspro.com)
- [ThermoSysPro license](https://github.com/ThermoSysPro/ThermoSysPro/blob/master/LICENSE.md)
