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

add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., 0.04, 0.0)); id="LeftFinger")
add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., -0.04, 0.0)); id="RightFinger")
add_coordinate!(robot, FrameOrigin("panda_hand"); id="HandBase")

vms = VirtualMechanismSystem("RobotHandover", robot)
vm = vms.virtual_mechanism
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.5,  -0.5, 0.4))); id="LeftFingerTarget")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.5, -0.42, 0.4))); id="RightFingerTarget")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.5, -0.46, 0.5))); id="HandBaseTarget")

add_coordinate!(vms, CoordDifference(".robot.LeftFinger", ".virtual_mechanism.LeftFingerTarget"); id="L pos error")
add_coordinate!(vms, CoordDifference(".robot.RightFinger", ".virtual_mechanism.RightFingerTarget"); id="R pos error")
add_coordinate!(vms, CoordDifference(".robot.HandBase", ".virtual_mechanism.HandBaseTarget"); id="H pos error")

target_rot = AxisAngle(SVector(0., 1., 0.), Float64(π))
add_coordinate!(vms, QuaternionAttitude(".robot.panda_hand", target_rot);        id="TCP orientation")
add_component!(vms, TanhSpring("L pos error"; max_force=7.0, stiffness=5000.0); id="L spring")
add_component!(vms, LinearDamper(25.0, "L pos error"); id="L damper")
add_component!(vms, TanhSpring("R pos error"; max_force=7.0, stiffness=5000.0); id="R spring")
add_component!(vms, LinearDamper(25.0, "R pos error"); id="R damper")
K_rot = SMatrix{3, 3}(100., 0., 0., 0., 200., 0., 0., 0., 300.)
add_component!(vms, LinearSpring(K_rot, "TCP orientation");       id="Angular Spring")
D_rot = SMatrix{3, 3}(1, 0., 0., 0., 2, 0., 0., 0., 3)
add_component!(vms, LinearDamper(D_rot, "TCP orientation");       id="Angular Damper");
# add_component!(vms, TanhSpring("H pos error"; max_force=10.0, stiffness=10000.0); id="H spring")
# add_component!(vms, LinearDamper(25.0, "H pos error"); id="H damper")
# K = SMatrix{3, 3}(100., 0., 0., 0., 100., 0., 0., 0., 100.)
# add_component!(vms, LinearSpring(K, "R pos error");       id="R spring")
# add_component!(vms, LinearSpring(K, "L pos error");       id="L spring")
# add_component!(vms, LinearSpring(K, "H pos error");       id="H spring")d

function f_setup(cache)
    LeftFinger_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.LeftFingerTarget")
    RightFinger_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.RightFingerTarget")
    HandBase_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.HandBaseTarget")
    return (LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id)
end

function f_control(cache, target_positions, t, setup_ret, extra)
    LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id = setup_ret
    # LeftFinger_coord = cache[LeftFinger_coord_id].coord_data.val[] = SVector(target_positions[1], target_positions[2], target_positions[3])
    # RightFinger_coord = cache[RightFinger_coord_id].coord_data.val[] = SVector(target_positions[4], target_positions[5], target_positions[6])
    # HandBase_coord = cache[HandBase_coord_id].coord_data.val[] = SVector(target_positions[7], target_positions[8], target_positions[9])
    nothing 
end

cvms = compile(vms)

# qᵛ = Float64[]
# with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 7, 14, 3) do connection
#     ros_vm_controller(connection, cvms, qᵛ; f_control, f_setup, E_max=15.0)
# end

tspan = (0., 20π)
dcache = new_dynamics_cache(compile(vms))
q = ([0.0, 0.3, 0.0, -1.8, 0.0, π/2, 0.0], Float64[])
q̇ = zero_q̇(dcache.vms)
g = VMRobotControl.DEFAULT_GRAVITY
prob = get_ode_problem(dcache, g, q, q̇, tspan)
@info "Simulating robot handover sample."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-3, reltol=1e-3); # Low tol to speed up simulation

# ## Plotting
# We create a figure with two scenes, for two different camera angles. We plot the robot, the
# targets, the TCPs, and the obstacles in the scene.
# We use observables for the time and the kinematics cache, which will be updated in the function
# `animate_robot_odesolution`, causing any plots that depend upon these observables to be updated.
fig = Figure(size = (720, 720), figure_padding=0)
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
tcp_scatter_kwargs = (;
    color=:blue, 
    marker=:x, 
    markersize=15, 
    label="TCPs",
    transparency=true ## Avoid ugly white outline artefact on markers
)


## Show robot
# robotvisualize!(ls, plotting_kcache;)
robotsketch!(ls, plotting_kcache, scale=0.3, linewidth=2.5, transparency=true)

## Label target and TCP
target_1_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.LeftFingerTarget")
target_2_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.RightFingerTarget")
target_3_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.HandBaseTarget")

l_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.LeftFinger")
r_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.RightFinger")
h_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.HandBase")

scatter!(ls, plotting_kcache, [target_1_pos_id, target_2_pos_id, target_3_pos_id]; target_scatter_kwargs...)
scatter!(ls, plotting_kcache, [l_tcp_pos_id, r_tcp_pos_id, h_tcp_pos_id]; tcp_scatter_kwargs...)

savepath = "docs/src/assets/random_trial.mp4"
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t, fastforward=1.0, fps=20)