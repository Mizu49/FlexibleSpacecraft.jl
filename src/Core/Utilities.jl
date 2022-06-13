module Utilities

export yamlread2matrix

"""
    yamlread2matrix

function that converts the direct read data from YAML file into the Matrix

# Argument

* `x::AbstractVector`: direct read data from YAML
* `size::Tuple{<:Int, <:Int}`: size of the desired matrix
"""
@inline function yamlread2matrix(x::AbstractVector, size::Tuple{<:Int, <:Int})
    return Matrix(transpose(reshape(x, reverse(size))))
end

end
