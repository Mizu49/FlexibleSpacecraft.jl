# Orbit simulation

`FlexibleSpacecraft.jl` provides the basic simulation feature for orbital motion of the spacecraft. Orbital motion is the cause for the gravitational disturbance torque, which is a dominant disturbance for the spacecraft with flexible and large structures. Also the orbital motion affects the directional control of the spacecraft attitude.

## Orbital models

* Circular orbit
* Elliptical orbit (To be implemented!)

## Parameter settings for orbital simulation

Parameter setting files should be prepared with YAML format. The basic setting file will be as follows:

```yaml
# Specify what type of configuration file you will use
property: orbit

# Specify the model used in the simulation of orbital motion
DynamicsModel: CircularOrbit

# Orbital elements
OrbitalElements:
    right ascention: 0
    inclination: 0
    semimajor axis: 421e3
    eccentricity: 0
    argument of perigee: 0
    true anomaly at epoch: 0

# Note for this parameter setting
OrbitInfo: Test orbit parameters
```

The [orbital elements](https://en.wikipedia.org/wiki/Orbital_elements) are the basic parameter for the attitude calculation. Specify the parameters under which your spacecraft will orbit.
