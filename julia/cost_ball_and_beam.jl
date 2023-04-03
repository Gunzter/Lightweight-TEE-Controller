if !isdefined(Main, :JitterTime)
    using JitterTime
end # if
if !isdefined(Main, :Statistics)
    using Statistics
end # if
if !isdefined(Main, :LinearAlgebra)
    using LinearAlgebra
end # if
if !isdefined(Main, :CSV)
    using CSV
end # if
if !isdefined(Main, :DataFrames)
    using DataFrames
end # if

const h             = 0.01
const plant_id      = 1
const sampler_id    = 2
const ctrler_id     = 3
const actuator_id   = 4

function create_plant()
    A = [0 0 0;
         -10 0 0;
         0 1 0]
    B = [4.5; 0; 0]
    C = [1 0 0;
         0 0 1]
    D = [0; 0]

    Qc = [5 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1]
    Rc = [405 0 0;
          0 0 0;
          0 0 0]

    return ContinuousSystem(plant_id, A, B, C, D, actuator_id, Rc, Qc)
end # function
function create_ctrler()
    F = [0.708674068771462 0.054032841745482 0.041070828641229;
         0.010551467211443 0.997298357912726 -0.218582048762962;
         0.003634293174562 0.009990994526376 0.933598148167317]
    G = [0.151618322577206 0.000768177974279;
         -0.103566086778876 0.216490098432186;
         -0.004111008573120 0.066394878664914]
    H = [2.432765994985051 -1.200729816566263 -0.561996667238517]
    K = [0.671847530600102 -0.367759035328323]
    
    return DiscreteSystem(ctrler_id, F, G, H, K, sampler_id)
end # function
function create_sampler()
    return DiscreteSystem(sampler_id, Matrix{Int}(I, 2, 2), plant_id)
end # function
function create_actuator()
    return DiscreteSystem(actuator_id, -1, ctrler_id)
end # function

function get_normalisation_cost(exectimes::Vector{T}) where {T <: Real}
    # Create PeriodicJitterTimeSystem struct
    sys_per = PeriodicJitterTimeSystem(JitterTimeSystem([create_plant(), create_sampler(), create_ctrler(), create_actuator()]))
    calcDynamics!(sys_per)

    # start periodic analysis
    beginPeriodicAnalysis!(sys_per)
    for i in 1:length(exectimes)
        iterate_cost_ree!(sys_per, exectimes[i], T(0))
    end
    endPeriodicAnalysis!(sys_per)

    # Set cost to zero and covariance matrix to stationary one
    sys_per.J = 0.0
    sys_per.dJdt = 0.0
    sys_per.P = sys_per.Pper

    # iterate again to acquire cost
    for i in 1:length(exectimes)
        iterate_cost_ree!(sys_per, exectimes[i], T(0))
    end

    return sys_per.J
end # function

function iterate_cost_tee!(N::PeriodicJitterTimeSystem, exectime::T, delta_i::T, delta_o::T) where {T <: Real}
    passTime!(N, delta_i)
    # reset clock
    execSys!(N, sampler_id)
    execSys!(N, ctrler_id)
    passTime!(N, exectime)
    execSys!(N, actuator_id) # NO LET
    passTime!(N, delta_o)
    if delta_o + exectime < h
        passTime!(N, h - delta_o - exectime)
    end # if
end # function

""" exectimes, delta_i, delta_o has to be in seconds"""
function find_real_cost_tee(exectimes::Vector{T}, delta_i::Vector{T}, delta_o::Vector{T}, normalisation::Bool = true) where {T <: Real}
    # Create PeriodicJitterTimeSystem struct
    sys_per = PeriodicJitterTimeSystem(JitterTimeSystem([create_plant(), create_sampler(), create_ctrler(), create_actuator()]))
    calcDynamics!(sys_per)

    # Get normalisation cost
    J_bar = 1
    if normalisation
        J_bar = get_normalisation_cost(exectimes)
    end # if

    # start periodic analysis
    beginPeriodicAnalysis!(sys_per)
    for i = 1:length(exectimes)
        iterate_cost_tee!(sys_per, exectimes[i], delta_i[i], delta_o[i])
    end # for
    endPeriodicAnalysis!(sys_per)

    # Set cost to zero and covariance matrix to stationary one
    sys_per.J = 0.0
    sys_per.dJdt = 0.0
    sys_per.P = sys_per.Pper

    # Iterate sequence once more to get proper costs
    for i = 1:length(exectimes)
        iterate_cost_tee!(sys_per, exectimes[i], delta_i[i], delta_o[i])
    end # for

    # return momentary cost and final cost
    return sys_per.J / J_bar
end # function

function iterate_cost_ree!(N::PeriodicJitterTimeSystem, exectime::T, delta_m::T) where {T <: Real}
    execSys!(N, sampler_id)
    passTime!(N, delta_m)
    execSys!(N, ctrler_id)
    passTime!(N, exectime)
    execSys!(N, actuator_id)
    if exectime + delta_m < h
        passTime!(N, h - exectime - delta_m)
    end
end # function
function find_real_cost_ree(exectimes::Vector{T}, delta_m::Vector{T}, normalisation::Bool = true) where {T <: Real}
    # Create PeriodicJitterTimeSystem struct
    sys_per = PeriodicJitterTimeSystem(JitterTimeSystem([create_plant(), create_sampler(), create_ctrler(), create_actuator()]))
    calcDynamics!(sys_per)

    # Get normalisation cost
    J_bar = 1
    if normalisation
        J_bar = get_normalisation_cost(exectimes)
    end # if

    # start periodic analysis
    beginPeriodicAnalysis!(sys_per)
    for i = 1:length(exectimes)
        iterate_cost_ree!(sys_per, exectimes[i], delta_m[i])
    end # for
    endPeriodicAnalysis!(sys_per)

    # Set cost to zero and covariance matrix to stationary one
    sys_per.J = 0.0
    sys_per.dJdt = 0.0
    sys_per.P = sys_per.Pper

    # Iterate sequence once more to get proper costs
    for i = 1:length(exectimes)
        iterate_cost_ree!(sys_per, exectimes[i], delta_m[i])
    end # for

    # return momentary cost and final cost
    return sys_per.J / J_bar
end # function

#function cost_vec_ree(delta_m_vec::Vector{T}; exectime::T = h/5) where {T <: Real}
#    J_vec = similar(delta_m_vec)
#    for (i, delta_m) in enumerate(delta_m_vec)
#        @info delta_m
#        J_vec[i] = find_cost_ree(1, malicious = true, delta_m = delta_m, exectime = exectime)
#    end # for
#    return J_vec
#end # function

""" returns tuple: (exectimes, delta_i, delta_o) """
function read_data_tee(experiment::String)
    if !(experiment in ["tee_exp_I", "tee_exp_II", "tee_exp_III"])
       throw(LoadError("Couldn't find experiment: $(experiment)"))
   end # if

   f = CSV.File("$(experiment).csv")
   return f[:exec_time]./10^6, f[:tau_i]./10^6, f[:tau_o]./10^6
end # function

""" returns tuple: (exectimes, delta_m) """
function read_data_ree(experiment::String)
    if !(experiment in ["ree_exp_000", "ree_exp_025", "ree_exp_050", "ree_exp_075", "ree_exp_100", "ree_exp_125"])
       throw(LoadError("Couldn't find experiment: $(experiment)"))
   end # if

   f = CSV.File("$(experiment).csv")
   return f[:exec_time]./10^6, f[:malicious_wait_time]./10^6
end # function

""" Save data from tee experiments """
function save_data_tee()
    df = DataFrame()
    experiments = ["tee_exp_I", "tee_exp_II", "tee_exp_III"]
    for (i, exp) in enumerate(experiments)
        e, delta_i, delta_o = read_data_tee(exp)
        J_norm = find_real_cost_tee(e, delta_i, delta_o, true)
        J = find_real_cost_tee(e, delta_i, delta_o, false)
        append!(df, DataFrame(Experiment = i, J_normalised = J_norm, J = J))
    end
    CSV.write("results_tee_costs.csv", df)
end # function

""" Save data from ree experiments """
function save_data_ree()
    df = DataFrame()
    experiments = ["ree_exp_000", "ree_exp_025", "ree_exp_050", "ree_exp_075", "ree_exp_100", "ree_exp_125"]
    for exp in experiments
        i = exp[end-2:end]
        e, delta_m = read_data_ree(exp)
        e .-= delta_m
        J_norm = try 
            find_real_cost_ree(e, delta_m, true)
        catch
            Inf
        end
        J = try 
            find_real_cost_ree(e, delta_m, false)
        catch
            Inf
        end
        append!(df, DataFrame(Experiment = i, J_normalised = J_norm, J = J))
    end
    CSV.write("results_ree_costs.csv", df)
end # function
