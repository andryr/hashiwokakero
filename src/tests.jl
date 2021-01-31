include("generation.jl")
include("resolution.jl")

"""
Run tests and return results aggregated by a parameter

Arguments:
solveFunc: solving function, take a grid as argument and must return if the solution is optimal and the time taken to solve
paramFunc: takes a grid as argument, must an integer
"""
function tests(solveFunc, paramFunc)
    sizes = [4, 9, 16, 25]

    results = Dict{Int, Vector{Tuple{Bool, Float64}}}()

    # Dummy solve, required as first call to solve function is usually much slower
    solveFunc(generateInstance(2))

    # For each grid size considered
    for (k, n) in enumerate(sizes)

        # Generate and solve 10 instances
        for instance in 1:10
            println("Instance n=$n")
            g = generateInstance(n)
                    
            isOptimal, elapsedTime = solveFunc(g)
            param = paramFunc(g)
            if !haskey(results, param)
                results[param] = Vector{Tuple{Bool, Float64}}()
            end
            push!(results[param], (isOptimal, elapsedTime))
        end
    end

    optRatio = Dict{Int, Float64}()
    means = Dict{Int, Float64}()

    # Aggregate results
    for (param, vec) in results
       optRatio[param], means[param] = aggregate(results[param])
    end

    return optRatio, means
end

function aggregate(res::Vector{Tuple{Bool, Float64}})
    optRatio = sum(function(x)
        isOpt, _ = x
        return isOpt
        end, res) / length(res)
    means = sum(function(x)
        _, elapsedTime = x
        return elapsedTime
        end, res) / length(res)
    return optRatio, means
end

function gurobiSolveTest(g)
    isOptimal, elapsedTime, _ = ipSolve(g, Gurobi.Optimizer)
    return isOptimal, elapsedTime
end

function gurobiSolveTest2(g)
    isOptimal, elapsedTime, _ = ipSolve2(g, Gurobi.Optimizer)
    return isOptimal, elapsedTime
end

function cbcSolveTest(g)
    isOptimal, elapsedTime, _ = ipSolve(g, Cbc.Optimizer)
    return isOptimal, elapsedTime
end

function heuristicSolveTest(g)
    start = time()
    isOptimal, _ = heuristicSolve(g)
    return isOptimal, time() - start
end

function paramSize(grid)
    return size(grid, 1)
end

function paramVerticeCount(grid)
    return sum(grid .> 0)
end

function plotResults(res::Dict{Int, Float64}; kwargs...)
    plot(sort(pairs(res)); kwargs...)
end

function plotResults(res1::Dict{Int, Float64}, res2::Dict{Int, Float64}; kwargs...)
    pairs1 = collect(sort(pairs(res1)))
    pairs2 = collect(sort(pairs(res2)))
    x = map(function(x)
    key, val = x
    return key
    end, pairs1)
    y = map(function(x)
    key, val = x
    return val
    end, pairs1)
    z = map(function(x)
    key, val = x
    return val
    end, pairs2)
    plot(x, [y  z]; kwargs...)
end