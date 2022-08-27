# Documentation for the CLI

`FlexibleSpacecraft.jl` offers a useful CLI tools to accelerate your development of the spacecraft. This documentation illustrates the basic settings and configurations of the CLI tool.

## Pre-process of the simulation parameters

You need a parameter setting file to run simulation from the CLI tool. Visit the [Parameter configuration](@ref) to learn more about the parameter setting format.
## Commands and basic usage

`evalspacecraft` is the topmost basic CLI command for `FlexibleSpacecraft.jl`. You need to type the subcommand to specify what you want to do with `FlexibleSpacecraft.jl`.

```text
$ evalspacecraft <command>
```

Subcommands are listed as follows:

* `update`: update and rebuild the `FlexibleSpacecraft.jl`. Recommended to use this subcommand at the first time you use `FlexibleSpacecraft.jl`
* `run <configfilepath>`: run simulation based on the given parameter settings and configurations
* `clear`: remove package `FlexibleSpacecraft.jl`

You can also use the following flags:

* `-h, --help`: show help
* `-V, --version`: show version information

### Example

```text
$ evalspacecraft -V
```

Suppose you have the parameter setting file `spacecraft.yml` in your current working directory. You can run simulation with the predefined parameters with following command. 

```text
$ evalspacecraft run spacecraft.yml --save
```

`--save` is a flag that specifies wheather to save the simulation data or not. False by default.

To test the CLI system, please try the shellscript `/test/main.sh` in the GitHub repository. This will help you to find out how to use our CLI system.
