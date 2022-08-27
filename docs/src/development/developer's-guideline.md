# Developer's Guideline

We appreciate your contributions to our project! Please read the guideline for developers.

## Branching Model

We employ the following branching model. Please try to follow this!

| Branch naming  | Description                                                                                                               | Branch off from: | Merge back into:                               |
| -------------- | ------------------------------------------------------------------------------------------------------------------------- | ---------------- | ---------------------------------------------- |
| `main`         | Latest stable release with version tag.                                                                                   | None             | None                                           |
| `develop`    | Latest development build. Newly developed features are merged to this branch.                                             | `main`           | `release-****` or `main` with `--no-ff` option |
| `dev-****`     | Feature development branch. Development of new feature should be on this branch. Contributors are encouraged to use this. | `development`    | `development` with `no-ff` option              |
| `release-****` | Preparation for next release will be done in this branch.                                                                 | `development`    | `main` and `develop` with `no-ff` option       |
| `hotfix-****`  | Bug fix in `main`                                                                                                         | `main`           | `main` and `development`                       |  |

`****` in branch naming is a short description of the development effort in that branch. It should be a lower camel case (e.g. `dev-differentialEquation`).

This branching model is inspired by [Vincent Driessen's Branching Model](https://nvie.com/posts/a-successful-git-branching-model/)

## Style Guide

**Please follow the [official style guide](https://docs.julialang.org/en/v1/manual/style-guide/) of JuliaLang!**

In addition, we are using the following naming conventions.

* `****2####()`: function that returns transformation matrix from `****` to `####`.  
  * eg. `ECI2BodyFrame()` returns transformation matrix from ECI frame to Spacecraft Body Fixed frame.
* `C_****2####`: variable that has transformation matrix from `****` to `####`.

## Documentation

Developers are encouraged to add [docstrings](https://docs.julialang.org/en/v1/manual/documentation/) in code. Especially when developing a new feature, be sure to explain it in detail and hopefully example code!

Documentation page is also good! Direct to `/docs/src/` to add new page in the `FlexibleSpacecraft.jl` Docs!

When using the information from external sources (like books and technical articles), be sure to add appropriate reference information. We recommend following the [Reference Style and Format](https://www.aiaa.org/publications/journals/reference-style-and-format) of the American Institute of Aeronautics and Astronautics (AIAA).
