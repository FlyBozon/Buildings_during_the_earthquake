# This solution was created with the help of ChatGPT and CloudIO AI. 
# The State Model was designed by me and is included in the code. 
# Initially, the GIF generation was implemented in one library but later refactored for better interactivity and real-time animation capabilities. 
# The solution also utilizes resources from Stack Overflow, Julia documentation, and the JuliaLang Discourse forum.

using GLMakie, GeometryBasics
using LinearAlgebra, DifferentialEquations

const GRAVITY = 9.81
const DAMPING_RATIO = 0.03 #chatGPT told that damping ratio for building is typically 5%
#google tells that for concrete structures it is about 5%, for steel - 2%

obs_build_p= Observable(Vector{Tuple{Float64, Float64, Float64}}())
time_obs = Observable(0.0)
running_simulation =Observable(true)
selected_p = Observable(nothing)
displacement_data = Observable(Vector{Tuple{Float64, Float64}}())

DEFAULT_NUM_FLOORS = 5
DEFAULT_FLOOR_STIFFNESS = 30e6
DEFAULT_FLOOR_MASS= 200000.0
DEFAULT_MAX_AMPLITUDE = 0.5
DEFAULT_WAVE_FREQUENCY =1.0
DEFAULT_EARTHQUAKE_DURATION = 10.0
FL_H=3.0  #height of the floor, wasnt constant but decided to add as it is also constant parameter, uses smts and can be easier changed

num_fls = Observable(DEFAULT_NUM_FLOORS)
fl_stiff =Observable(DEFAULT_FLOOR_STIFFNESS)
fl_mass = Observable(DEFAULT_FLOOR_MASS)
max_a =Observable(DEFAULT_MAX_AMPLITUDE)
w_freq= Observable(DEFAULT_WAVE_FREQUENCY)
eqke_dur = Observable(DEFAULT_EARTHQUAKE_DURATION)

#create a matrix from characteristic vector
#input - vector of characteristic; output - matrix of that characteristic
#same for damping and k, look at model stanowy, I will add it somewhere
#vector input: [b1, b2, b3]
#simple scheme for 3floors: [-b1-b2  b2    0 ]
#                           [b2    -b2-b3  b3]
#                           [0       b3   -b3]
function gen_char_matr(characteristic::Vector)
    n = length(characteristic)
# the length of vector is at the same time the size of the matrix (nxn)
    char_matrix = zeros(Float64, n, n)  # Initialize an n x n matrix with zeros - line from chatGPT
    for i in 1:n
        for j in 1:n
            if i == j
                # diagonal
                char_matrix[i, j] = -characteristic[i] - (i < n ? characteristic[i + 1] : 0)
            elseif i == j - 1
                #upper diagonal
                char_matrix[i, j] = characteristic[i + 1]
            elseif i == j + 1
                #lower diagonal
                char_matrix[i, j] = characteristic[i]
            end
        end
    end
    return char_matrix
end

#create a matrix from mass vector
#was thinking about putting it in the characteristics matrix, 
#but decided to make another function if later decide to create 
#some additional functionalities, and then can change that function nuch nore
#mass matrix: [m1 0   0]
#             [0  m2  0]
#             [0  0  m3]
#thinking about leave it like vector and work on vectors 
#for optimalization, as that is a diagonal matrix
function gen_mass_matr(mass::Vector)
    return Diagonal(mass)
end

function create_matrices(floors, stiffness, mass)
    # Mass matrix
    M = gen_mass_matr(fill(mass, floors))
    
    # Stiffness matrix
    K = gen_char_matr(fill(stiffness, floors - 1))
    
    #wikipedia Rayleigh Damping: C = αM + βK
    α = 0.01 
    β = 0.01 
    C = α * M + β * K
    #println(M, K, C)

    return M, K, C
end

#equation of motion M * x'' + C * x' + K * x = F(t)
function dyn_rest(M, K, C, ext_force, tspan)
    # State-space representation
    num_dofs = size(M, 1)
    A = [zeros(num_dofs, num_dofs) I;
         -inv(M) * K -inv(M) * C]
    B = [zeros(num_dofs); inv(M)]

    # ODE system
    function dynamics!(du, u, p, t)
        F = ext_force(t) 
        du .= A * u .+ B * F
    end

    #zero point
    u0 = zeros(2 * num_dofs)
    prob = ODEProblem(dynamics!, u0, tspan)
    sol = solve(prob, Tsit5())
    return sol
end

function earthquake_wave(time::Float64, z::Float64, max_a::Float64, w_freq::Float64, duration::Float64)
    shear_wave_speed = 3000.0
    phase_delay = z / shear_wave_speed
    if time > duration
        return 0.0
    else
        return max_a * sin(2π * w_freq * (time - phase_delay))
    end
end

function eqke_force(floors, time, max_a, w_freq, duration)
    forcing = zeros(Float64, floors)
    for i in 1:floors
        forcing[i] = earthquake_wave(time, i * FL_H, max_a, w_freq, duration)
    end
    return forcing
end

function create_building(floors::Int)
    building_p = Vector{Tuple{Float64, Float64, Float64}}()
    for i in 0:floors-1
        push!(building_p, (0.0, 0.0, i * FL_H))
        push!(building_p, (10.0, 0.0, i * FL_H))
        push!(building_p, (10.0, 10.0, i * FL_H))
        push!(building_p, (0.0, 10.0, i * FL_H))
    end
    return building_p
end

function sway_building!(time, floors, stiffness, mass, max_a, w_freq, duration)
    M, K, C = create_matrices(floors, stiffness, mass)

    function ext_force(t)
        eqke_force(floors, t, max_a, w_freq, duration)
    end

    tspan = (0.0, duration)
    sol = dyn_rest(M, K, C, ext_force, tspan)

    displacement = sol(time)[1:floors]

    updated_p = Vector{Tuple{Float64, Float64, Float64}}()
    original_p = create_building(floors)

    for i in 1:floors
        sway_x = displacement[i]
        sway_y = displacement[i]  
        torsion = 0.01 * displacement[i]

        for j in 1:4
            p = original_p[4 * (i - 1) + j]
            center_x, center_y = 5.0, 5.0
            rotated_x = (p[1] - center_x) * cos(torsion) - (p[2] - center_y) * sin(torsion) + center_x
            rotated_y = (p[1] - center_x) * sin(torsion) + (p[2] - center_y) * cos(torsion) + center_y
            push!(updated_p, (rotated_x + sway_x, rotated_y + sway_y, p[3]))
        end
    end

    obs_build_p[] = updated_p
end

function create_gui()
    fig = Figure(size = (800, 800))

    ax = LScene(fig[1, 1], scenekw = (show_axis = true,))
    colsize!(fig.layout, 1, Aspect(1, 1))
    resize_to_layout!(fig)

    obs_build_p[] = create_building(num_fls[])

    scatter_plot = scatter!(ax, lift(obs_build_p) do p
        map(p -> Point3f0(p...), p)
    end, markersize = 10, color = :blue)

    # asked chatgpt to draw walls, not really what I wanted, but looks kinda nice
    lines!(ax, lift(obs_build_p) do p
        line_coords = []
        for i in 1:num_fls[]
            floor_points = p[4 * (i - 1) + 1:4 * i]
            for j in 1:4
                start_point = Point3f0(floor_points[j]...)
                end_point = Point3f0(floor_points[mod(j, 4) + 1]...)
                push!(line_coords, start_point, end_point)
            end
            if i > 1
                prev_floor_points = p[4 * (i - 2) + 1:4 * (i - 1)]
                for j in 1:4
                    start_point = Point3f0(prev_floor_points[j]...)
                    end_point = Point3f0(floor_points[j]...)
                    push!(line_coords, start_point, end_point)
                end
            end
        end
        return line_coords
    end, color = :blue)  

    sliders = SliderGrid(
        fig[2, 1],
        (label = "Floors", range = 1:1:20, startvalue = num_fls[]),
        (label = "Stiffness", range = 10e6:1e6:100e6, startvalue = fl_stiff[]),
        (label = "Mass", range = 100000:10000:500000, startvalue = fl_mass[]),
        (label = "Max Amplitude", range = 0.1:0.1:2.0, startvalue = max_a[]),
        (label = "Wave Frequency", range = 0.5:0.1:5.0, startvalue = w_freq[]),
        (label = "Earthquake Duration", range = 1.0:1.0:30.0, startvalue = eqke_dur[])
    )
    slider_observables = [s.value for s in sliders.sliders]
    on(slider_observables[1]) do f
        num_fls[] = Int(f)
        obs_build_p[] = create_building(num_fls[])
        selected_p[] = nothing
    end
    on(slider_observables[2]) do s
        fl_stiff[] = s
    end
    on(slider_observables[3]) do m
        fl_mass[] = m
    end
    on(slider_observables[4]) do a
        max_a[] = a
    end
    on(slider_observables[5]) do w
        w_freq[] = w
    end
    on(slider_observables[6]) do d
        eqke_dur[] = d
    end

#reset inspiration from https://discourse.julialang.org/t/glmakie-reset-all-sliders-inside-slidergrid-via-button/94555
    reset_params_button = Button(fig[3, 1]; label = "Reset Parameters")
    on(reset_params_button.clicks) do n
        for slider in sliders.sliders
            set_close_to!(slider, slider.startvalue[])
        end

        num_fls[] = DEFAULT_NUM_FLOORS
        fl_stiff[] = DEFAULT_FLOOR_STIFFNESS
        fl_mass[] = DEFAULT_FLOOR_MASS
        max_a[] = DEFAULT_MAX_AMPLITUDE
        w_freq[] = DEFAULT_WAVE_FREQUENCY
        eqke_dur[] = DEFAULT_EARTHQUAKE_DURATION

        obs_build_p[] = create_building(num_fls[])
        selected_p[] = nothing
    end

    reset_simulation_button = Button(fig[3, 2], label = "Reset Simulation")
    on(reset_simulation_button.clicks) do _
        time_obs[] = 0.0
        obs_build_p[] = create_building(num_fls[])
        displacement_data[] = Vector{Tuple{Float64, Float64}}()
    end

    @async while true
        if running_simulation[]
            sway_building!(
                time_obs[],  #current t, from it count current state
                num_fls[],
                fl_stiff[],
                fl_mass[],
                max_a[],
                w_freq[],
                eqke_dur[]
            )
            time_obs[] += 0.05 
            sleep(0.05)
        end
    end

    fig
end

create_gui()
