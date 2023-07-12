# Metroscope Modeling Library

## Content

 The Metroscope Modeling Library contains modelica modules for industrial components such as pumps, turbines, heat exchangers, to build steady-state digital twins of industrial process. The associated documentation contains explanation of the library equations. 

## Usage

- When creating a Digital Twin you should only use official releases of `Metroscope Modeling Library`. Official releases can be downloaded from [here](https://github.com/Metroscope-dev/metroscope-modeling-library/releases).
- In your model, you should add a line in `annotation`, containing the following : `uses(MetroscopeModelingLibrary(version(="3.0.1"))` in which you replace the version number (`"3.0.1"`) by the one you use. That way you will get a warning if you use your model with a wrong version of the library.
- If you need additionnal features, please ask for them using [issues](https://github.com/Metroscope-dev/metroscope-modeling-library/issues)

## Documentation

Library documentation is available in this [public Notion space](https://metroscope.notion.site/Metroscope-Modeling-Library-Documentation-50c8703c294446059d3b4a70d6ae4a71?pvs=4). 
You can use comments in the documentation to ask questions about the library. Please do not include any proprieraty information. In order to comment the page, you need to create a Notion account.

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

Notify the MML team for a review of your PR. 

## Attribution notice

Licensed by Metroscope under the Modelica License 2
Copyright © 2023, Metroscope.

This Modelica package is free software and the use is completely at your own risk; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see Modelica.UsersGuide.ModelicaLicense2 or visit http://www.modelica.org/licenses/ModelicaLicense2.

Metroscope Modeling Library is a Derivative Work of ThermoSysPro:
- [ThermoSysPro home page](https://thermosyspro.com)
- [ThermoSysPro license](https://github.com/ThermoSysPro/ThermoSysPro/blob/master/LICENSE.md)
