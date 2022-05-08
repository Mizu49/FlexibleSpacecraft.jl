"""
    Structures

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module Structures

using Reexport, YAML

include("SpringMass.jl")
@reexport using .SpringMass

export setstructure

"""
    setstructure

API function to define the model to
"""
function setstructure(configfilepath::String)

    lawread = YAML.load_file(configfilepath)

    if haskey(lawread, "modeling") == false
        throw(ErrorException("`model` is undefined in configuration file `$configfilepath`"))
    end

    if lawread["modeling"] == "spring-mass"
        (structureparams, structuresimmodel) = defmodel(lawread)
    else
        throw(ErrorException("no matching modeling method for \"$(lawread["modeling"])\""))
    end

    return (structureparams, structuresimmodel)
end

end
