# Developer's Guideline

We appreciate your contributions to our project! Please read the guideline for developers.

## Branching Model

We employ the following branching model. Please try to follow this!

| Branch naming  | Description                                                                                                               | Branch off from: | Merge back into:                               | 
| -------------- | ------------------------------------------------------------------------------------------------------------------------- | ---------------- | ---------------------------------------------- | 
| `main`         | Latest stable release with version tag.                                                                                   | None             | None                                           | 
| `dev-build`    | Latest development build. Newly developed features are merged to this branch.                                             | `main`           | `release-****` or `main` with `--no-ff` option | 
| `dev-****`     | Feature development branch. Development of new feature should be on this branch. Contributors are encouraged to use this. | `development`    | `development` with `no-ff` option              | 
| `release-****` | Preparation for next release will be done in this branch.                                                                 | `development`    | `main` and `develop` with `no-ff` option       | 
| `hotfix-****`  | Bug fix in `main`                                                                                                         | `main`           | `main` and `development`                       |  | 

`****` in branch naming is a short description of the development effort in that branch. It should be a lower camel case (e.g. `dev-differentialEquation`).

This branching model is inspired by [Vincent Driessen's Branching Model](https://nvie.com/posts/a-successful-git-branching-model/)

## Style Guide

**Please follow the [official style guide](https://docs.julialang.org/en/v1/manual/style-guide/) of JuliaLang!**

Especially be sure to code using the style guide below:

- Indentation
    - Use four spaces per indentation level!
- Naming conventions
    - modules and type names use capitalization and camel case: `module SparseArrays`, `struct UnitRange`.
    - functions are lowercase (`maximum`, `convert`) and, when readable, with multiple words squashed together (`isequal`, `haskey`). When necessary, use underscores as word separators. Underscores are also used to indicate a combination of concepts (`remotecall_fetch` as a more efficient implementation of `fetch(remotecall(...))`) or as modifiers.
    - conciseness is valued, but avoid abbreviation (`indexin` rather than `indxin`) as it becomes difficult to remember whether and how particular words are abbreviated.
