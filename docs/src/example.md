# Example

This documentation page is an example and quick start guide for the use of `FlexibleSpacecraft.jl`

## Example script and files

Example script `main.jl` is found in the `/test` directory. Configuration and parameter setting file is preferred for the simulation, and these files should be in YAML format. Detailed formatting for the parameter settings is found in the [Parameter configurations]().

You need the following files:

* `main.jl`: the main script
* `orbit.yml`: main configuration for orbital parameters
* `spacecraft.yml`: parameter settings for spacecraft configuration

These files are found on the `test` directory in the GitHub repository. Run the `main.jl`, and you will get the simulation result. By default, the software provides the data set of simulation results and plots of those data. It also gives you a GIF animation of the spacecraft attitude.
