# This file contains methods to generate a data set of instances (i.e., sudoku grids)
include("io.jl")
include("utils.jl")

"""
Generate a game instance with n vertices

Argument
- n: minmum number of vertices
"""
function generateInstance(n::Int)
    E = Vector{Tuple{Int,Int}}()

    V, Vh, Vv = generateVertices(n)

    # Create all possible edges without caring about intersections
    for (i, L) in Vh
        for l = 1:length(L) - 1
            k1, _, _ = L[l]
            k2, _, _ = L[l + 1]
            edge = (k1, k2)
            push!(E, edge)
        end
    end

    for (j, L) in Vv
        for l = 1:length(L) - 1
            k1, _, _ = L[l]
            k2, _, _ = L[l + 1]
            edge = (k1, k2)
            push!(E, edge)
        end
    end
    
    while (inter = oneIntersection(V, E)) != Nothing
        (i, j, k1, k2, l1, l2) = inter
        push!(V, (i, j))
        k = length(V)
        filter!(function (x)
            return x ∉ [(k1, k2), (l1, l2)]
        end, E)
        push!(E, (k, k1))
        push!(E, (k, k2))
        push!(E, (k, l1))
        push!(E, (k, l2))
    end

    # Compute values for each vertice
    values = fill(0, length(V))
    for (k1, k2) in E
        val = ceil(Int, rand() * 2)
        values[k1] += val
        values[k2] += val 
    end
    
    # Now we can build a grid representing the game instance
    grid = fill(0, n, n)
    for k = 1:length(V)
        val = values[k]
        i, j = V[k]
        grid[i,j] = val
    end
    return grid
end 


function generateVertices(n::Int)
    V = Vector{Tuple{Int,Int}}(undef, n)
    S = Set{Tuple{Int,Int}}()
    Vh = Dict{Int,Vector{Tuple{Int,Int,Int}}}()
    Vv = Dict{Int,Vector{Tuple{Int,Int,Int}}}()
    # First vertice is chosen randomly
    (i, j) = (ceil(Int, rand() * n), ceil(Int, rand() * n))
    push!(S, (i, j))
    V[1] = (i, j)
    addEntry(i, (1, i, j), Vh)
    addEntry(j, (1, i, j), Vv)
    for k = 2:n
        (i, j) = (ceil(Int, rand() * n), ceil(Int, rand() * n))
        while (i, j) in S || !haskey(Vh, i) && !haskey(Vv, j) # if (i,j) was already chosen or if it is not aligned with any vertice of V
            (i, j) = (ceil(Int, rand() * n), ceil(Int, rand() * n))
        end
        push!(S, (i, j))
        V[k] = (i, j)
        addEntry(i, (k, i, j), Vh)
        addEntry(j, (k, i, j), Vv)
    end

    for (i, L) in Vh
        sort!(L, by = function (x)
            k, i, j = x
            return j
        end)
    end
    for (j, L) in Vv
        sort!(L, by = function (x)
            k, i, j = x
            return i
        end)
    end
    return V, Vh, Vv
end


"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()

    sizes = [4, 9, 16, 25, 50]
    for n in sizes
        for instance in 1:10
            fileName = "../data/instance_n" * string(n) *  "_" * string(instance) * ".txt"
            if !isfile(fileName)
                println("-- Generating file " * fileName)
                saveInstance(generateInstance(n), fileName)
            end 
        end
    end
    
end



