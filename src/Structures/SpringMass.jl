"""
    SpringMass

submodule that contains all features for spring-mass modeling of flexible appendages of spacecraft
"""
module SpringMass

using LinearAlgebra, StaticArrays

"""
    PhysicalSystem

Representation of the structural system in physical coordinate

## Fields

`𝐌::AbstractMatrix`: mass matrix in physical coordinate
`𝐂::AbstractMatrix`: damping matrix in physical coordinate
`𝐊::AbstractMatrix`: stiffness matrix in physical coordinate
"""
struct PhysicalSystem
    dim::Integer

    𝐌::AbstractMatrix
    𝐂::AbstractMatrix
    𝐊::AbstractMatrix

    # Constructor
    PhysicalSystem(𝐌::AbstractMatrix, 𝐂::AbstractMatrix, 𝐊::AbstractMatrix) = begin
        dim = size(𝐌, 1)
        # convert to SMatrix for fast computation
        𝐌 = SMatrix{dim, dim}(𝐌)
        𝐂 = SMatrix{dim, dim}(𝐂)
        𝐊 = SMatrix{dim, dim}(𝐊)

        new(dim, 𝐌, 𝐂, 𝐊)
    end
end

struct SpringMassModel
    # degrees of freedom (DOF) of the structure
    DOF
    # dimension of the control input vector
    dimcontrolinput
    # dimension of the disturbance input vector
    dimdistinput
    # transformation matrix. i.e. x = 𝚽𝛈, modal coordinates are mass-normalized
    𝚽
    # modal damping matrix
    𝚵
    # coupling matrix with the attitude dynamics (time derivative of the angular velocity vector)
    𝐃
    # disturbance input matrix
    𝐅
end

struct StateSpace
    𝐀 # system matrix
    𝐁 # control input matrix
    𝐄c # input matrix for the coupling part (subscript c represents coupling)
    𝐄d # input matrix for the disturbance input (subscript d represents disturbance)

    StateSpace(model::SpringMassModel) = begin
        𝐀 = [
            zeros(model.DOF) I
            -𝚽^2 -2*𝚵*𝚽
        ]
        𝐁 = [
            zeros(model.DOF, model.dimcontrolinput)
            model.controlinputmatrix
        ]
        𝐄c = [zeros(model.DOF, 3); 𝐃]
        𝐄d = [zeros(model.DOF, model.dimdistinput); model.𝐅]
    end
end

"""
    _mode_decomposition(𝐌::AbstractMatrix, 𝐂::AbstractMatrix, 𝐊::AbstractMatrix)::Tuple

return tuple of the modal transformation matrix and modal damping matrix for the mass-normalized modal coordinates
"""
function _mode_decomposition(𝐌::AbstractMatrix, 𝐂::AbstractMatrix, 𝐊::AbstractMatrix)::Tuple

    # dimension of the structure
    dim = size(𝐌, 1)

    # Eigen value decomposition
    # https://docs.julialang.org/en/v1/stdlib/LinearAlgebra/#LinearAlgebra.eigvecs
    𝚽 = eigvecs(𝐌, 𝐊)

    # mass normalization
    mr = zeros(dim)
    for ind = 1:dim
        mr[ind] = 𝚽[:, ind]' * 𝐌 * 𝚽[:, ind]
        𝚽[:, ind] = sqrt(1/mr[ind])*𝚽[:, ind]
    end

    # natural angular frequency matrix
    # 𝛀 is defined to be diagonal matrix of natural angular frequency, not its squared value
    𝛀 = sqrt(transpose(𝚽) * 𝐊 * 𝚽)

    # calculate modal stiffness matrix Kr = omega^2
    kr = zeros(dim)
    for idx = 1:dim
        kr[idx] = 𝛀[idx, idx]^2
    end

    # modal damping ratio
    cr = zeros(dim)
    xi_r = zeros(dim)
    for idx = 1:dim
        cr[idx] = transpose(𝚽[:, idx]) * 𝐂 * 𝚽[:, idx]
        xi_r[idx] = cr[idx]/(2 * sqrt(mr[idx] * kr[idx]))
    end
    𝚵 = diagm(xi_r)

    # sort matrix with natural angular frequency
    sortidx = sortperm(diag(𝛀))
    𝛀 = 𝛀[sortidx, sortidx]
    𝚽 = 𝚽[:, sortidx]
    𝚵 = 𝚵[sortidx, sortidx]

    return (𝚽, 𝛀, 𝚵)
end

end
