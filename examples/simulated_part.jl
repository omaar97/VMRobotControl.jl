using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl
using FileIO, UUIDs
using Printf
using Sockets
using StaticArrays
using .Threads 
using VMRobotControl

using VMRobotControl:
    DEFAULT_GRAVITY,
    DEFAULT_F_SETUP,
    DEFAULT_F_CONTROL,
    robot_ndof

try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end
cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
# module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
# module_path = pathof(VMRobotControl)
# robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/panda_with_gripper.urdf"), cfg)
robot = parseURDF("URDFs/franka_description/urdfs/panda_with_gripper.urdf", cfg)

joint_limits = cfg.joint_limits

add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)

for (i, τ_coulomb) in zip(1:7, [5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0])
    β = 1e-1
    limits = joint_limits["panda_joint$i"]
    isnothing(limits) && continue
    @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
    add_coordinate!(robot, JointSubspace("panda_joint$i");    id="J$i")
    add_deadzone_springs!(robot, 50.0, (limits.lower+0.1, limits.upper-0.1), "J$i")
    add_component!(robot, TanhDamper(τ_coulomb, β, "J$i");         id="JointDamper$i")
end;

points_distance_gripper = 0.9
points_distance_TCP = 0.5
add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., points_distance_gripper/2, 0.0)); id="LeftFinger")
add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., -points_distance_gripper/2, 0.0)); id="RightFinger")
add_coordinate!(robot, FramePoint("panda_hand", SVector(0., 0., -(points_distance_TCP - 0.1034))); id="HandBase") # 0.1034: distance between gripper and TCP
add_coordinate!(robot, FrameOrigin("panda_hand"); id="pandaHand")

# add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.95, 0.5))); id="LeftFingerTarget")
# add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.05, 0.5))); id="RightFingerTarget")
# add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.5, 1.0))); id="HandBaseTarget")

X = SVector(1., 0., 0.)
Y = SVector(0., 1., 0.)
Z = SVector(0., 0., 1.)

vm = Mechanism{Float64}("IntermediateMechanism")

# F0 = root_frame(vm)
# F1 = add_frame!(vm; id="L1_frame")
# F2 = add_frame!(vm; id="L2_frame")
# F3 = add_frame!(vm; id="L3_frame")

# J1 = Prismatic(X)
# J2 = Prismatic(Y)
# J3 = Prismatic(Z)

# add_joint!(vm, J1; parent=F0, child=F1, id="J1")
# add_joint!(vm, J2; parent=F1, child=F2, id="J2")
# add_joint!(vm, J3; parent=F2, child=F3, id="J3")

# # add_coordinate!(vm, ConstCoord(SVector(0.3, -0.95, 0.5)); id="LeftFingerTarget")
# # add_coordinate!(vm, ConstCoord(SVector(0.3, -0.05, 0.5)); id="RightFingerTarget")
# # add_coordinate!(vm, ConstCoord(SVector(0.3, -0.5, 1.0)); id="HandBaseTarget")

# add_coordinate!(vm, FramePoint(F3, SVector(0.3, -0.95, 0.3)); id="VMLeftFinger")
# add_coordinate!(vm, FramePoint(F3, SVector(0.3, -0.05, 0.3)); id="VMRightFinger")
# add_coordinate!(vm, FramePoint(F3, SVector(0.3, -0.5, 0.8)); id="VMHandBase")
# add_component!(vm, PointMass(0.5, "VMLeftFinger"); id="VLFMass")
# add_component!(vm, PointMass(0.5, "VMRightFinger"); id="VRFMass")
# add_component!(vm, PointMass(0.5, "VMHandBase"); id="VHBMass")

# add_gravity_compensation!(vm, VMRobotControl.DEFAULT_GRAVITY)

# vms = VirtualMechanismSystem("RobotHandover", robot, vm)

# add_coordinate!(vms, ReferenceCoord(Ref(SVector(0.3, -0.95, 0.5))); id="LeftFingerTarget")
# add_coordinate!(vms, ReferenceCoord(Ref(SVector(0.3, -0.05, 0.5))); id="RightFingerTarget")
# add_coordinate!(vms, ReferenceCoord(Ref(SVector(0.3, -0.5, 1.0))); id="HandBaseTarget")

# add_component!(vms, TanhSpring("VLF pos error"; max_force=5.0, stiffness=50.0); id="VLF spring")
# add_component!(vms, LinearDamper(20.0, "VLF pos error"); id="VLF damper")
# add_component!(vms, TanhSpring("VRF pos error"; max_force=5.0, stiffness=50.0); id="VRF spring")
# add_component!(vms, LinearDamper(20.0, "VRF pos error"); id="VRF damper")
# add_component!(vms, TanhSpring("VHB pos error"; max_force=5.0, stiffness=100.0); id="VHB spring")
# add_component!(vms, LinearDamper(20.0, "VHB pos error"); id="VHB damper")

# add_coordinate!(vms, CoordDifference(".virtual_mechanism.VMLeftFinger", "LeftFingerTarget"); id="VLF pos error")
# add_coordinate!(vms, CoordDifference(".virtual_mechanism.VMRightFinger", "RightFingerTarget"); id="VRF pos error")
# add_coordinate!(vms, CoordDifference(".virtual_mechanism.VMHandBase", "HandBaseTarget"); id="VHB pos error")

# add_coordinate!(vms, CoordDifference(".robot.LeftFinger", ".virtual_mechanism.VMLeftFinger"); id="L pos error")
# add_coordinate!(vms, CoordDifference(".robot.RightFinger", ".virtual_mechanism.VMRightFinger"); id="R pos error")
# add_coordinate!(vms, CoordDifference(".robot.HandBase", ".virtual_mechanism.VMHandBase"); id="H pos error")

# add_component!(vms, TanhSpring("L pos error"; max_force=2.0, stiffness=100.0); id="L spring")
# add_component!(vms, LinearDamper(1.0, "L pos error"); id="L damper")
# add_component!(vms, TanhSpring("R pos error"; max_force=2.0, stiffness=100.0); id="R spring")
# add_component!(vms, LinearDamper(1.0, "R pos error"); id="R damper")
# add_component!(vms, TanhSpring("H pos error"; max_force=2.0, stiffness=200.0); id="H spring")
# add_component!(vms, LinearDamper(1.0, "H pos error"); id="H damper")

vm = Mechanism{Float64}("IntermediateMechanism")

F0 = root_frame(vm)
F1 = add_frame!(vm; id="L1_frame")
F2 = add_frame!(vm; id="L2_frame")
F3 = add_frame!(vm; id="L3_frame")

J1 = Prismatic(X)
J2 = Prismatic(Y)
J3 = Prismatic(Z)

add_joint!(vm, J1; parent=F0, child=F1, id="VJ1")
add_joint!(vm, J2; parent=F1, child=F2, id="VJ2")
add_joint!(vm, J3; parent=F2, child=F3, id="VJ3")
# add_coordinate!(vm, JointSubspace("VJ1");    id="VJC1")
# add_coordinate!(vm, JointSubspace("VJ2");    id="VJC2")
# add_coordinate!(vm, JointSubspace("VJ3");    id="VJC3")
# add_component!(vm, LinearDamper(1.0, "VJC1");         id="VMDamper1")
# add_component!(vm, LinearDamper(1.0, "VJC2");         id="VMDamper2")
# add_component!(vm, LinearDamper(1.0, "VJC3");         id="VMDamper3")

# add_coordinate!(vm, ConstCoord(SVector(0.3, -0.95, 0.5)); id="LeftFingerTarget")
# add_coordinate!(vm, ConstCoord(SVector(0.3, -0.05, 0.5)); id="RightFingerTarget")
# add_coordinate!(vm, ConstCoord(SVector(0.3, -0.5, 1.0)); id="HandBaseTarget")

add_coordinate!(vm, FramePoint(F3, SVector(0.0, 0.0, 0.0)); id="VMLeftFinger")
# add_coordinate!(vm, FramePoint(F3, SVector(0.3, -0.05, 0.3)); id="VMRightFinger")
# add_coordinate!(vm, FramePoint(F3, SVector(0.3, -0.5, 0.8)); id="VMHandBase")
add_component!(vm, PointMass(10.0, "VMLeftFinger"); id="VLFMass")
# add_component!(vm, PointMass(50.0, "VMRightFinger"); id="VRFMass")
# add_component!(vm, PointMass(50.0, "VMHandBase"); id="VHBMass")

add_gravity_compensation!(vm, VMRobotControl.DEFAULT_GRAVITY)

vms = VirtualMechanismSystem("RobotHandover", robot, vm)

# add_coordinate!(vms, ReferenceCoord(Ref(SVector(0.3, -0.95, 0.5))); id="LeftFingerTarget")
# add_coordinate!(vms, ReferenceCoord(Ref(SVector(0.3, -0.05, 0.5))); id="RightFingerTarget")
# add_coordinate!(vms, ReferenceCoord(Ref(SVector(0.3, -0.5, 1.0))); id="HandBaseTarget")

add_coordinate!(vms, ConstCoord(SVector(0.3, -0.95, 0.5)); id="LeftFingerTarget")
# add_coordinate!(vms, ConstCoord(SVector(0.3, -0.05, 0.5)); id="RightFingerTarget")
# add_coordinate!(vms, ConstCoord(SVector(0.3, -0.5, 1.0)); id="HandBaseTarget")

add_coordinate!(vms, CoordDifference("LeftFingerTarget", ".virtual_mechanism.VMLeftFinger"); id="VLF pos error")
# add_coordinate!(vms, CoordDifference("RightFingerTarget", ".virtual_mechanism.VMRightFinger"); id="VRF pos error")
# add_coordinate!(vms, CoordDifference("HandBaseTarget", ".virtual_mechanism.VMHandBase"); id="VHB pos error")

add_component!(vms, TanhSpring("VLF pos error"; max_force=10.0, stiffness=100.0); id="VLF spring")
add_component!(vms, LinearDamper(50.0, "VLF pos error"); id="VLF damper")
# add_component!(vms, TanhSpring("VRF pos error"; max_force=10.0, stiffness=100.0); id="VRF spring")
# add_component!(vms, LinearDamper(10.0, "VRF pos error"); id="VRF damper")
# add_component!(vms, TanhSpring("VHB pos error"; max_force=10.0, stiffness=100.0); id="VHB spring")
# add_component!(vms, LinearDamper(10.0, "VHB pos error"); id="VHB damper")

add_coordinate!(vms, CoordDifference(".robot.LeftFinger", ".virtual_mechanism.VMLeftFinger"); id="L pos error")
# add_coordinate!(vms, CoordDifference(".robot.RightFinger", ".virtual_mechanism.VMRightFinger"); id="R pos error")
# add_coordinate!(vms, CoordDifference(".robot.HandBase", ".virtual_mechanism.VMHandBase"); id="H pos error")

# add_component!(vms, TanhSpring("L pos error"; max_force=5.0, stiffness=100.0); id="L spring")
# add_component!(vms, LinearDamper(1.0, "L pos error"); id="L damper")
# add_component!(vms, TanhSpring("R pos error"; max_force=0.1, stiffness=100.0); id="R spring")
# add_component!(vms, LinearDamper(1.0, "R pos error"); id="R damper")
# add_component!(vms, TanhSpring("H pos error"; max_force=0.1, stiffness=200.0); id="H spring")
# add_component!(vms, LinearDamper(1.0, "H pos error"); id="H damper")



# K = SMatrix{3, 3}(100., 0., 0., 0., 100., 0., 0., 0., 100.)
# add_component!(vms, LinearSpring(K, "R pos error");       id="R spring")
# add_component!(vms, LinearSpring(K, "L pos error");       id="L spring")
# add_component!(vms, LinearSpring(K, "H pos error");       id="H spring")

# function f_setup(cache)
#     LeftFinger_coord_id = get_compiled_coordID(cache, "LeftFingerTarget")
#     # RightFinger_coord_id = get_compiled_coordID(cache, "RightFingerTarget")
#     # HandBase_coord_id = get_compiled_coordID(cache, "HandBaseTarget")
#     VMLeftFinger_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.VMLeftFinger")
#     # VMHandBase_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.VMHandBase")
#     # return (LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id, VMHandBase_coord_id, VMLeftFinger_coord_id)
#     return (LeftFinger_coord_id, VMLeftFinger_coord_id)
# end

# function f_control(cache, target_positions, t, setup_ret, extra)
#     # LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id, VMHandBase_coord_id, VMLeftFinger_coord_id = setup_ret
#     LeftFinger_coord_id, VMLeftFinger_coord_id = setup_ret
#     print(configuration(cache, VMLeftFinger_coord_id))
#     print("\n")
#     print(configuration(cache, LeftFinger_coord_id))
#     print("\n----------- \n")
#     # cache[LeftFinger_coord_id].coord_data.val[] = SVector(target_positions[1], target_positions[2], target_positions[3])
#     # cache[RightFinger_coord_id].coord_data.val[] = SVector(target_positions[4], target_positions[5], target_positions[6])
#     # cache[HandBase_coord_id].coord_data.val[] = SVector(target_positions[7], target_positions[8], target_positions[9])
#     nothing 
# end

# disturbance_func(t) = mod(t, 6) < 3 ? SVector(0., 0., 0.) : SVector(0., 5.0, 5.0)

# f_setup(cache) = get_compiled_coordID(cache, ".robot.pandaHand")
# function f_control(cache, t, args, extra)
#     tcp_pos_coord_id = args
#     F = disturbance_func(t)
#     uᵣ, uᵥ = get_u(cache)
#     z = configuration(cache, tcp_pos_coord_id)
#     J = jacobian(cache, tcp_pos_coord_id)
#     mul!(uᵣ, J', F)
#     nothing
# end

# function f_setup(cache)
#     # LeftFinger_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.LeftFingerTarget")
#     # RightFinger_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.RightFingerTarget")
#     # HandBase_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.HandBaseTarget")
#     # return (LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id)
#     LeftFinger_coord_id = get_compiled_coordID(cache, "LeftFingerTarget")
#     return (LeftFinger_coord_id)
# end

# function f_control(cache, t, setup_ret, extra)
#     LeftFinger_coord_id = setup_ret
#     # LeftFinger_coord = cache[LeftFinger_coord_id].coord_data.val[] = SVector(target_positions[1], target_positions[2], target_positions[3])
#     # RightFinger_coord = cache[RightFinger_coord_id].coord_data.val[] = SVector(target_positions[4], target_positions[5], target_positions[6])
#     # HandBase_coord = cache[HandBase_coord_id].coord_data.val[] = SVector(target_positions[7], target_positions[8], target_positions[9])W 
#     nothing
# end

cvms = compile(vms)

# qᵛ = Float64[]
# with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 7, 14, 3) do connection
#     ros_vm_controller(connection, cvms, qᵛ; f_control, f_setup, E_max=15.0)
# end

tspan = (0., 10)
dcache = new_dynamics_cache(compile(vms))
q = ([0.0, 0.3, 0.0, -1.8, 0.0, π/2, 0.0], [0.0, 0.0, 0.0])
q̇ = zero_q̇(dcache.vms)
g = VMRobotControl.DEFAULT_GRAVITY
prob = get_ode_problem(dcache, g, q, q̇, tspan; f_setup)
@info "Simulating robot handover sample."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-3, reltol=1e-3); # Low tol to speed up simulation

# ## Plotting
# We create a figure with two scenes, for two different camera angles. We plot the robot, the
# targets, the TCPs, and the obstacles in the scene.
# We use observables for the time and the kinematics cache, which will be updated in the function
# `animate_robot_odesolution`, causing any plots that depend upon these observables to be updated.
fig = Figure(size = (1920, 1080), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0., 0., 0.3]
cam.eyeposition[] = [1.5, 0., 0.3]
plotting_t = Observable(0.0)
plotting_kcache = Observable(new_kinematics_cache(compile(vms)))

target_scatter_kwargs = (;
    color=:green, 
    marker=:+, 
    markersize=15, 
    label="Targets",
    transparency=true ## Avoid ugly white outline artefact on markers
)

target2_scatter_kwargs = (;
    color=:black, 
    marker=:+, 
    markersize=15, 
    label="Targets",
    transparency=true ## Avoid ugly white outline artefact on markers
)

tcp_scatter_kwargs = (;
    color=:blue, 
    marker=:x, 
    markersize=15, 
    label="TCPs",
    transparency=true ## Avoid ugly white outline artefact on markers
)
# tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.pandaHand")
# tcp_pos = map(plotting_kcache) do kcache
#     Point3f(configuration(kcache, tcp_pos_id))
# end
# force = map(t -> 0.01 * Vec3f(disturbance_func(t)), plotting_t)
# arrowsize = map(f -> 0.1*(f'*f)^(0.25), force)
# arrows!(ls, map(p -> [p], tcp_pos), map(f -> [f], force); color = :red, arrowsize)

## Show robot
robotvisualize!(ls, plotting_kcache;)
robotsketch!(ls, plotting_kcache, scale=0.3, linewidth=2.5, transparency=true)

## Label target and TCP
target_1_pos_id = get_compiled_coordID(plotting_kcache[], "LeftFingerTarget")
# target_2_pos_id = get_compiled_coordID(plotting_kcache[], "RightFingerTarget")
# target_3_pos_id = get_compiled_coordID(plotting_kcache[], "HandBaseTarget")

target_1V_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.VMLeftFinger")
# target_2V_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.VMRightFinger")
# target_3V_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.VMHandBase")

l_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.LeftFinger")
# r_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.RightFinger")
# h_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.HandBase")

# scatter!(ls, plotting_kcache, [target_1_pos_id, target_2_pos_id, target_3_pos_id]; target_scatter_kwargs...)
# scatter!(ls, plotting_kcache, [target_1V_pos_id, target_2V_pos_id, target_3V_pos_id]; target2_scatter_kwargs...)
# scatter!(ls, plotting_kcache, [l_tcp_pos_id, r_tcp_pos_id, h_tcp_pos_id]; tcp_scatter_kwargs...)

scatter!(ls, plotting_kcache, [target_1_pos_id]; target_scatter_kwargs...)
scatter!(ls, plotting_kcache, [target_1V_pos_id]; target2_scatter_kwargs...)
scatter!(ls, plotting_kcache, [l_tcp_pos_id]; tcp_scatter_kwargs...)

savepath = "docs/src/assets/random_trial.mp4"
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t, fastforward=1.0, fps=20)