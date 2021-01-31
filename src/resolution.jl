# This file contains methods to solve an instance (heuristically or with CPLEX)
import Gurobi
import Cbc
using Plots

include("generation.jl")

TOL = 0.00001

"""
Solve an instance using integer programming
"""
function ipSolve(grid::Array{Int,2}, optimizer)
    # Start a chronometer
    start = time()

    m, n = size(grid)

    V = Vector{Tuple{Int,Int,Int}}()
    for i = 1:m
        for j = 1:n
            val = grid[i, j]
            if val > 0
                push!(V, (val, i, j))
            end
        end
    end

    K = length(V)

    # Create the model
    m = Model(optimizer)

    @variable(m, x[1:K,1:K], Bin, Symmetric)
    @variable(m, y[1:K,1:K], Int, Symmetric)

    # No self-loops
    @constraint(m, [k = 1:K], x[k,k] == 0)
    @constraint(m, [k = 1:K], y[k,k] == 0)



    for k1 in 1:K, k2 in 1:K
        (_, i1, j1) = V[k1]
        (_, i2, j2) = V[k2]
        # if vertices are not on the same horizontal/vertical axis, no edge between them
        if i1 != i2 && j1 != j2
            @constraint(m, x[k1, k2] == 0)
        end
    end

    for k1 in 1:K
        (_, ik1, jk1) = V[k1]
        for l1 in 1:k1
            (_, il1, jl1) = V[l1]
            if k1 == l1
                continue
            end
            for k2 in 1:k1
                (_, ik2, jk2) = V[k2]

                for l2 in 1:k2
                    # if edges (k1, l1) and (k2, l2) are not compatible add a constraint to the model
                    (_, il2, jl2) = V[l2]

                    if k2 == l2
                        continue
                    end

                    if (k1, l1) == (k2, l2) || (l1, k1) == (k2, l2)
                        continue
                    end

                    if k1 == k2 && il1 != il2 && jl1 != jl2 || k1 == l2 && il1 != ik2 && jl1 != jk2 || l1 == k2 && ik1 != il2 && jk1 != jl2 || l1 == l2 && ik1 != ik2 && jk1 != jk2
                        continue
                    end

                    R1 = (max(ik1, ik2, il1, il2) - min(ik1, ik2, il1, il2) + 1) * (max(jk1, jk2, jl1, jl2) - min(jk1, jk2, jl1, jl2) + 1)
                    R2 = (max(ik1, il1) - min(ik1, il1) + 1) * (max(jk2, jl2) - min(jk2, jl2) + 1)
                    R3 = (max(ik2, il2) - min(ik2, il2) + 1) * (max(jk1, jl1) - min(jk1, jl1) + 1)
                    if R1 == max(R2, R3)
                        @constraint(m, x[k1,l1] + x[k2, l2] <= 1)
                    end
                end
            end
        end
    end
    
    # Ensure that x and y are symmetric
    # @constraint(m, [i = 1:K, j = 1:K], x[i,j] == x[j, i])
    # @constraint(m, [i = 1:K, j = 1:K], y[i,j] == y[j, i])

    # Ensure coherence between x and y + bridges are single or double
    @constraint(m, [i = 1:K, j = 1:K], x[i,j] <= y[i, j])
    @constraint(m, [i = 1:K, j = 1:K], y[i,j] <= 2 * x[i,j])

    # Numbers of briges are respected
    @constraint(m, [i = 1:K], sum(y[i, j] for j = 1:K) == V[i][1])

    # Connectivity
    @objective(m, Max, sum(x[i, j] for i = 1:K, j = 1:K))

    # Solve the model
    optimize!(m)

    elapsedTime = time() - start

    status = JuMP.primal_status(m)
    if status != MOI.NO_SOLUTION
        sol = round.(Int, value.(y))
        return JuMP.termination_status(m) == MOI.OPTIMAL, elapsedTime, sol, V
    else
        return false, elapsedTime, Nothing, Nothing
    end
end

"""
Solve an instance using integer programming, without the connectivity constraint
"""
function ipSolve2(grid::Array{Int,2}, optimizer)

    m, n = size(grid)

    V = Vector{Tuple{Int,Int,Int}}()
    for i = 1:m
        for j = 1:n
            val = grid[i, j]
            if val > 0
                push!(V, (val, i, j))
            end
        end
    end

    K = length(V)

    # Create the model
    m = Model(optimizer)

    @variable(m, x[1:K,1:K], Bin, Symmetric)
    @variable(m, y[1:K,1:K], Int, Symmetric)

    # No self-loops
    @constraint(m, [k = 1:K], x[k,k] == 0)

    for k1 in 1:K, k2 in 1:K
        (_, i1, j1) = V[k1]
        (_, i2, j2) = V[k2]
        # if vertices are not on the same horizontal/vertical axis, no edge between them
        if i1 != i2 && j1 != j2
            @constraint(m, x[k1, k2] == 0)
        end
    end

    for k1 in 1:K, l1 in 1:K, k2 in 1:K, l2 in 1:K
        # if edges (k1, l1) and (k2, l2) are not compatible add a constraint to the model
        (_, ik1, jk1) = V[k1]
        (_, il1, jl1) = V[l1]
        (_, ik2, jk2) = V[k2]
        (_, il2, jl2) = V[l2]

        if k1 == l1 || k2 == l2
            continue
        end

        if (k1, l1) == (k2, l2) || (l1, k1) == (k2, l2)
            continue
        end

        if k1 == k2 && il1 != il2 && jl1 != jl2 || k1 == l2 && il1 != ik2 && jl1 != jk2 || l1 == k2 && ik1 != il2 && jk1 != jl2 || l1 == l2 && ik1 != ik2 && jk1 != jk2
            continue
        end

        R1 = (max(ik1, ik2, il1, il2) - min(ik1, ik2, il1, il2) + 1) * (max(jk1, jk2, jl1, jl2) - min(jk1, jk2, jl1, jl2) + 1)
        R2 = (max(ik1, il1) - min(ik1, il1) + 1) * (max(jk2, jl2) - min(jk2, jl2) + 1)
        R3 = (max(ik2, il2) - min(ik2, il2) + 1) * (max(jk1, jl1) - min(jk1, jl1) + 1)
        if R1 == max(R2, R3)
            @constraint(m, x[k1,l1] + x[k2, l2] <= 1)
        end
    end

    
    # Ensure that x and y are symmetric
    # @constraint(m, [i = 1:K, j = 1:K], x[i,j] == x[j, i])
    # @constraint(m, [i = 1:K, j = 1:K], y[i,j] == y[j, i])

    # Ensure coherence between x and y + bridges are single or double
    @constraint(m, [i = 1:K, j = 1:K], x[i,j] <= y[i, j])
    @constraint(m, [i = 1:K, j = 1:K], y[i,j] <= 2 * x[i,j])

    # Numbers of briges are respected
    @constraint(m, [i = 1:K], sum(y[i, j] for j = 1:K) == V[i][1])

    # Connectivity
    #@objective(m, Max, sum(x[i, j] for i = 1:K, j = 1:K))

    println("Start optimizing...")
    # Start a chronometer
    start = time()

    # Solve the model
    optimize!(m)

    elapsedTime = time() - start
    println("Done!")

    status = JuMP.primal_status(m)
    if status != MOI.NO_SOLUTION
        sol = round.(Int, value.(y))
        return isConnected(V, sol), elapsedTime, sol, V
    else
        return false, elapsedTime, Nothing, Nothing
    end
end

"""
Heuristically solve an instance
"""
function heuristicSolve(grid::Array{Int,2}, kMax = 10000, initT = 1000.0, lambda = 0.999)
    m, n = size(grid)
    V = Vector{Tuple{Int,Int,Int}}()
    Vh = Dict{Int,Vector{Tuple{Int,Int,Int}}}()
    Vv = Dict{Int,Vector{Tuple{Int,Int,Int}}}()

    k = 1
    for i = 1:m
        for j = 1:n
            val = grid[i, j]
            if val > 0
                push!(V, (val, i, j))
                addEntry(i, (k, i, j), Vh)
                addEntry(j, (k, i, j), Vv)
                k += 1
            end
        end
    end

    
    # Solve using simulated annealing
    # state = set of edges
    state = Vector{Tuple{Int,Int,Int}}()
    optState = state
    T = initT
    E = energy(V, state)
    Eopt = E
    Ehist = [E]
    k = 1
    while k <= kMax
        sk = randomNeighbor(V, Vh, Vv, state)
        Ek = energy(V, sk)
        if Ek < E || rand() < exp(-(Ek - E) / T)
            state = sk
            E = Ek
            push!(Ehist, E)
            if E < Eopt
                Eopt = E
                optState = state
            end
        end
        T = lambda * T
        k += 1
    end

    p = length(V)
    edgeMatrix = fill(0, p, p)
    for (v, k1, k2) in optState
        edgeMatrix[k1, k2] = v
        edgeMatrix[k2, k1] = v
    end

    # Check if solution is valid
    isValid = true
    weights = fill(0, p)
    for (v, k1, k2) in optState
        weights[k1] += v
        weights[k2] += v
    end

    for k = 1:p
        v, i, j = V[k]
        if v != weights[k]
            isValid = false
            break
        end
    end

    if !isConnected(V, optState)
        isValid = false
    end

    if intersections(V, optState) > 0
        isValid = false
    end

    return isValid, edgeMatrix, V, Ehist
end

function energy(V::Vector{Tuple{Int,Int,Int}}, E::Vector{Tuple{Int,Int,Int}})
    p = length(V)
    
    conflicts = intersections(V, E)

    weights = fill(0, p)
    for (v, k1, k2) in E
        weights[k1] += v
        weights[k2] += v
    end

    weightDiffSum = 0
    for k = 1:p
        v, i, j = V[k]
        weightDiffSum += (weights[k] - v)^2
    end
    return conflicts  + weightDiffSum^2 - (length(E))
end

function randomNeighbor(V::Vector{Tuple{Int,Int,Int}}, Vh::Dict{Int,Vector{Tuple{Int,Int,Int}}}, Vv::Dict{Int,Vector{Tuple{Int,Int,Int}}}, E::Vector{Tuple{Int,Int,Int}})
    newE = deepcopy(E)
    m = length(Vh)
    n = length(Vv)
    nbEdges = length(E)
    nbVertices = length(V)

    p1 = 0 # probability to modify an edge
    p2 = 0 # probability to remove an edge
    # probability to add an edge is 1 - p1 - p2

    if 1 <= nbEdges < nbVertices - 1
        p1 = 0
        p2 = 0
    elseif nbVertices - 1 <= nbEdges
        p1 = 0.5
        p2 = 0.25
    end

    r = rand()
    if r < p1 # modify an edge value
        k = ceil(Int, rand()*length(newE))
        val, k1, k2 = newE[k]
        if val == 1
            val = 2
        else
            val = 1
        end
        newE[k] = (val, k1, k2)
    elseif r < p1 + p2 # delete an edge
            deleteat!(newE, ceil(Int, rand() * length(newE)))
    else # create a new edge
        k1 = ceil(Int, rand() * length(V))
        _, i1, j1 = V[k1]
        Lh = Vh[i1]
        Lv = Vv[j1]
        neighbors = Vector{Int}()
        lh = 1
        lv = 1
        for l = 1:length(Lh)
            if Lh[l][1] == k1
                lh = l
                break
            end
        end
        for l = 1:length(Lv)
            if Lv[l][1] == k1
                lv = l
                break
            end
        end

        if lh >= 2
            push!(neighbors, Lh[lh - 1][1])
        end
        if lh <= length(Lh) - 1
            push!(neighbors, Lh[lh + 1][1])
        end
        if lv >= 2
            push!(neighbors, Lv[lv - 1][1])
        end
        if lv <= length(Lv) - 1
            push!(neighbors, Lv[lv + 1][1])
        end

        k2 = neighbors[ceil(Int, rand() * length(neighbors))]
        push!(newE, (ceil(Int, rand() * 2), k1, k2))
        newE = unique(function (x)
            (v, k1, k2) = x
            if k1 <= k2
                return (k1, k2)
            else
                return (k2, k1)
            end
        end, newE)
    end
    return newE
end

"""
Solve all the instances contained in "../data" through CPLEX and heuristics

The results are written in "../res/cplex" and "../res/heuristic"

Remark: If an instance has previously been solved (either by cplex or the heuristic) it will not be solved again
"""
function solveDataSet()

    dataFolder = "../data/"
    resFolder = "../res/"

    # Array which contains the name of the resolution methods
    resolutionMethod = ["gurobi", "cbc", "heuristic"]

    # Array which contains the result folder of each resolution method
    resolutionFolder = resFolder .* resolutionMethod

    # Create each result folder if it does not exist
    for folder in resolutionFolder
        if !isdir(folder)
            mkdir(folder)
        end
    end
            
    global isOptimal = false
    global solveTime = -1

    # For each instance
    # (for each file in folder dataFolder which ends by ".txt")
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        grid = readInputFile(dataFolder * file)

        # For each resolution method
        for methodId in 1:size(resolutionMethod, 1)
            
            outputFile = resolutionFolder[methodId] * "/" * file

            # If the instance has not already been solved by this method
            if !isfile(outputFile)
                
                fout = open(outputFile, "w")  

                resolutionTime = -1
                isOptimal = false
                
                if resolutionMethod[methodId] == "gurobi"
                    optimizer = Gurobi.Optimizer 
                    # Solve it and get the results
                    isOptimal, resolutionTime, sol, V = ipSolve(grid, optimizer)
                elseif resolutionMethod[methodId] == "cbc"
                    optimizer = Cbc.Optimizer 
                    # Solve it and get the results
                    isOptimal, resolutionTime, sol, V = ipSolve(grid, optimizer)
                else # heuristic
                    # Start a chronometer
                    startingTime = time()
                    kMax = 1000
                    initT = 1000.0

                    while !isOptimal && kMax <= 1000000
                        # Solve it and get the results
                        isOptimal, sol, V = heuristicSolve(grid, kMax, initT, 0.999)

                        # Stop the chronometer
                        resolutionTime = time() - startingTime
                        kMax *= 10
                        initT *= 100
                    end
                end

                println(fout, "solveTime = ", resolutionTime)
                println(fout, "isOptimal = ", isOptimal)

                if isOptimal
                    writeSolution(fout, sol, V)
                end

                close(fout)
            end


            # Display the results obtained with the method on the current instance
            include(outputFile)
            println(resolutionMethod[methodId], " optimal: ", isOptimal)
            println(resolutionMethod[methodId], " time: " * string(round(solveTime, sigdigits = 2)) * "s\n")
        end         
    end 
end
