#!/bin/bash

echo "Testing the CLI of FlexibleSpacecraft.jl ..."

# run simulation with command
evalspacecraft run spacecraft.yml orbit2.yml disturbance.yml simconfig.yml initvalue.yml
