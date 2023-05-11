# How to contribute to MML ?

Anyone can contribute, and you are totally welcome to do so !

## You have found a bug / want a new feature in MML ?

- Create a new issue on the [MML issues page](https://github.com/Metroscope-dev/metroscope-modeling-library/issues)
- **Do not attach sensitive data to the GitHub issue you create**
- Pick the appropriate template (bug report or feature request) and fill it with your issue details

## You know how to fix a bug / add a new feature ?

- Pull the latest changes from `main` and create a branch on your local git installation (see [Git basic branching documentation](https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging) with a name corresponding to your bug/feature, e.g.: `fix-metroscopia-convergence` or `add-new-heat-exchanger-config`
- Do your developments on your branch
- Make sure your change do not break any test, that all components are still locally balanced
- Create a pull request on the [MML PR page](https://github.com/Metroscope-dev/metroscope-modeling-library/pulls)
- Fill in the PR template with your PR details, choose your reviewers and notify them so that they can review your work
- Once all the changes requested have been resolved you can squash and merge your changes into `main` branch.

## How to tag a new MML release

MML works with a `MAJOR`.`MINOR`.`PATCH` versionning :

* A change of MAJOR number means a structural or architectural change in the library
* A change of MINOR number means a modification of the library that will require an update of existing models
* A change of PATCH number means a modification of the library that keeps retrocompatibility with previous models

If you want to tag a new release, you have to :

1. Create a pull request on the [MML PR page](https://github.com/Metroscope-dev/metroscope-modeling-library/pulls) called `bump-to-vX.X.X`
2. Update the `CHANGELOG.md` file, putting all unreleased changes into a new section `vX.X.X`
3. Update the root `package.mo` `version` field in annotations, with your new version number
4. In the PR description, put the text of the future release notes, with all the breaking changes included in your release (using the changelog)
5. Ask for a review by the MML developping team and squash and merge once your PR is approved.
