using Plots

const GRAVITY = 9.81
const DAMPING = 0.05 #chatGPT told that damping ratio for building is typically 5%
#google tells that for concrete structures it is about 5%, for steel - 2%

mass = 5000.0 #later will change to matrix, now let it be constant/the same for each floor

function calculate_wn(stiffness::Float64, mass::Float64) #calculate natural frequency (wn=sqrt(k/m))
    return sqrt(stiffness/mass)
end

#another formula: damping=actualDamping/criticalDamping
function calculate_damping_coef(wn::Float64, mass::Float64) #calculate damping ratio (c=) formula from study.com
    return 2*mass*wn*DAMPING
end

#calculate the critical damping cd=2*sqrt(k*m) k-spring constant, m - mass
function calculate_cd(stiffness::Float64, mass::Float64)
    
end

function swaying(floor::Int, corner::Int, time::Float64, base_f::Float64, stiffness::Float64)
    if floor==1
        return(0.0,0.0,0.0)
    end

    wn=calculate_wn(stiffness, mass)
    damping_coef=calculate_damping_coef(wn, mass)

    decay=exp(damping_coef*time/(2*mass))

    sway_amplitude_x = 0.02 * floor * decay * sin(base_f * time + floor * 0.3)
    sway_amplitude_y = 0.015 * floor * decay * sin(base_f * time + floor * 0.5)
    vertical_movement = 0.005 * floor * decay * (cos(base_f * time + floor * 0.2))#+sin(base_f * time + floor * 0.8))

    return(sway_amplitude_x,sway_amplitude_y,vertical_movement)
end

function create_building(floors::Int)
    building_points = Vector{Tuple{Float64, Float64, Float64}}()
    for i in 0:floors-1
        push!(building_points, (0.0, 0.0, i))      
        push!(building_points, (1.0, 0.0, i))      
        push!(building_points, (1.0, 1.0, i))  
        push!(building_points, (0.0, 1.0, i))  
    end
    return building_points
end


function vibrating_building(points, time, floors, base_f::Float64,  stiffness::Float64)
    new_points=Vector{Tuple{Float64,Float64,Float64}}()
    for i in 0:floors-1
        for j in 1:4
            sway_x, sway_y, sway_z = swaying(i + 1, j, time, base_f, stiffness)
            p = points[4 * i + j]
            push!(new_points, (p[1] + sway_x, p[2] + sway_y, p[3] + sway_z))
        end
    end
    return new_points
end



function create_building_gif()
    floors = 5
    stiffness = 3000.0
    base_f = 1.5
    building = create_building(floors)
    n = 1000

    @gif for i in 1:n
        time = i * 0.1
        new_building = vibrating_building(building, time, floors, stiffness, base_f)

        Plots.plot()
        for j in 1:floors
            rect_x = [new_building[4 * j - 3][1], new_building[4 * j - 2][1], new_building[4 * j - 1][1], new_building[4 * j][1], new_building[4 * j - 3][1]]
            rect_y = [new_building[4 * j - 3][2], new_building[4 * j - 2][2], new_building[4 * j - 1][2], new_building[4 * j][2], new_building[4 * j - 3][2]]
            rect_z = [new_building[4 * j - 3][3], new_building[4 * j - 2][3], new_building[4 * j - 1][3], new_building[4 * j][3], new_building[4 * j - 3][3]]

            Plots.plot!(rect_x, rect_y, rect_z, lw = 2, legend = false, c = :blue, alpha = 0.5)
            
            if j > 1
                prev_floor = new_building[4 * (j - 2) + 1:4 * (j - 1)]
                current_floor = new_building[4 * (j - 1) + 1:4 * j]
                for k in 1:4
                    Plots.plot!([prev_floor[k][1], current_floor[k][1]],
                                [prev_floor[k][2], current_floor[k][2]],
                                [prev_floor[k][3], current_floor[k][3]], lw = 1.5, c = :black)
                end
            end
        end

        Plots.xlims!(-0.5, 1.5)
        Plots.ylims!(-0.5, 1.5)
        Plots.zlims!(0, floors + 1.5)
        Plots.xlabel!("X")
        Plots.ylabel!("Y")
        Plots.zlabel!("Height [Floor]")
        Plots.title!("3D Swaying Building")
    end every 5

    println("GIF created")
end

create_building_gif()
