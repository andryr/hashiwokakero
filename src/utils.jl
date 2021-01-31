# Util functions that are useful for both generation and solving

function isConnected(V::Vector{Tuple{Int, Int, Int}}, E::Vector{Tuple{Int, Int, Int}})
    n = length(V)
    adj = Vector{Vector{Int}}(undef, n)
    # build adjacency lists
    for k = 1:n
        adjK = Vector{Int}()
        for (_, k1, k2) in E
            if k == k1
                push!(adjK, k2)
            elseif k == k2
                push!(adjK, k1)
            end
        end
        adj[k] = adjK
    end
    return dfs(1, adj, fill(false, n)) == n
end

function isConnected(V::Vector{Tuple{Int, Int, Int}}, E::Array{Int,2})
    n = length(V)
    adj = Vector{Vector{Int}}(undef, n)
    # build adjacency lists
    for k = 1:n
        adj[k] = Vector{Int}()
    end
    for i=1:n
        for j=1:i-1
            if E[i,j] > 0
                push!(adj[i], j)
                push!(adj[j], i)
            end
        end
    end

    return dfs(1, adj, fill(false, n)) == n
end

function dfs(v::Int, adj::Vector{Vector{Int}}, mark::Vector{Bool})
    visited = 1
    mark[v] = true
    for w in adj[v]
        if mark[w]
            continue
        end
        visited += dfs(w, adj, mark)
    end
    return visited
end

function addEntry(key::Int, v::Tuple{Int,Int,Int}, V::Dict{Int,Vector{Tuple{Int,Int,Int}}})
    if haskey(V, key)
        push!(V[key], v)
    else
        L = Vector{Tuple{Int,Int,Int}}()
        push!(L, v)
        V[key] = L
    end
end

function removeEntry(key::Int, v::Tuple{Int,Int,Int}, V::Dict{Int,Vector{Tuple{Int,Int,Int}}})
    L = V[key]
    println(L, v)
    println(findfirst((x)->x == v, L))
    deleteat!(L, findfirst((x)->x == v, L))
    if length(L) == 0
        delete!(V, key)
    end
end

function intersections(V::Vector{Tuple{Int,Int,Int}}, E::Vector{Tuple{Int,Int,Int}})
    c = 0
    for (vk, k1, k2) in E, (vl, l1, l2) in E
        if (k1, k2) == (l1, l2)
            continue
        end
        _, ik1, jk1 = V[k1]
        _, ik2, jk2 = V[k2]
        _, il1, jl1 = V[l1]
        _, il2, jl2 = V[l2]
        if k1 == l1 && ik2 != il2 && jk2 != jl2 || k1 == l2 && ik2 != il1 && jk2 != jl1 || k2 == l1 && ik1 != il2 && jk1 != jl2 || k2 == l2 && ik1 != il1 && jk1 != jl1 
            continue
        end
        # if (k1,k2) and (l1,l2) don't intersect then R1 should be strictly greater than R2 and R3
        R1 = (max(ik1, ik2, il1, il2) - min(ik1, ik2, il1, il2) + 1) * (max(jk1, jk2, jl1, jl2) - min(jk1, jk2, jl1, jl2) + 1)
        R2 = (max(ik1, ik2) - min(ik1, ik2) + 1) * (max(jl1, jl2) - min(jl1, jl2) + 1)
        R3 = (max(il1, il2) - min(il1, il2) + 1) * (max(jk1, jk2) - min(jk1, jk2) + 1)
        if R1 == max(R2, R3)
            c += 1
        end
    end
    return c
end

function oneIntersection(V::Vector{Tuple{Int,Int}}, E::Vector{Tuple{Int,Int}})
    for (k1, k2) in E, (l1, l2) in E
        if (k1, k2) == (l1, l2) || (k2, k1) == (l1, l2)
            continue
        end
        ik1, jk1 = V[k1]
        ik2, jk2 = V[k2]
        il1, jl1 = V[l1]
        il2, jl2 = V[l2]
        if k1 == l1 && ik2 != il2 && jk2 != jl2 || k1 == l2 && ik2 != il1 && jk2 != jl1 || k2 == l1 && ik1 != il2 && jk1 != jl2 || k2 == l2 && ik1 != il1 && jk1 != jl1 
            continue
        end
        # if (k1,k2) and (l1,l2) don't intersect then R1 should be strictly greater than R2 and R3
        R1 = (max(ik1, ik2, il1, il2) - min(ik1, ik2, il1, il2) + 1) * (max(jk1, jk2, jl1, jl2) - min(jk1, jk2, jl1, jl2) + 1)
        R2 = (max(ik1, ik2) - min(ik1, ik2) + 1) * (max(jl1, jl2) - min(jl1, jl2) + 1)
        R3 = (max(il1, il2) - min(il1, il2) + 1) * (max(jk1, jk2) - min(jk1, jk2) + 1)
        if R1 == max(R2, R3)
            I = [ik1, ik2, il1, il2]
            J = [jk1, jk2, jl1, jl2]
            sort!(I)
            sort!(J)
            return (I[2], J[2], k1, k2, l1, l2)
        end
    end
    return Nothing
end