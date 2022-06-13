# Parameter configuration

`FlexibleSpacecraft.jl` offers an soft cording method to allow you to manage your simulation parameters and configurations. You need parameter setting and simulation configuration files to run your simulation with your desired setting. This documentation illustrates how to prepare your simulation configuration files.

## Configuration file format

Configuration file is organized with the YAML format. For more information about YAML, please visit [The Official YAML Web Site](https://yaml.org/).

```yaml
config:
    name: example spacecraft No. 2
    note: This model is for test purpose
    sampling time: 1e-2 # Sampling period of simulation (second)
    time length: 1000 # Time length of simulation (second)

initial value:
    quaternion: [0, 0, 0, 1]
    angular velocity: [0, 0, 0]


attitude dynamics:
    model: Linear coupling
    inertia: [
        1000, 0, 0,
        0, 1000, 0,
        0, 0, 1000
    ]

    coupling: [
        1, 0,
        0, 1,
        0, 0
    ]

disturbance:
    constant torque: [0.0, 0.0, 0.0]
    gravitational torque: false

Orbit:
    Dynamics model: Circular
    Orbital elements:
        right ascension: 0
        inclination: 0
        semimajor axis: 6763e3
        eccentricity: 0
        argument of perigee: 0
        true anomaly at epoch: 0

modeling: spring-mass

appendage:
    modeling: spring-mass
    system:
        coord: physical
        DOF: 2
        mass: [
            100, 0,
            0, 50
        ]
        stiffness: [
            6e4, -1e4,
            -1e4, 1e4
        ]
        damping:
            config: Rayleigh
            alpha: 0
            beta: 0.005
    coupling: [
            1, 0, 0,
            0, 1, 0,
        ]
    control input:
        dimension: 1
        coefficient: [
                5,
                0
            ]
    disturbance input:
        dimension: 1
        coefficient: [
                3,
                0
            ]
```

## Parameter setting API for the configuration file

Simply call the function `readparamfile`, giving the path of the configuration file.

```julia
paramfilepath = "./test/spacecraft2.yml"
(simconfig, attitudemodel, distconfig, initvalue, orbitinfo, strparam, strmodel) = readparamfile(paramfilepath)
```

Detailed docs for the parameter setting API is found on the [Parameter setting API](@ref)

This file is used for CLI feature. See [Documentation for the CLI](@ref) for more information
