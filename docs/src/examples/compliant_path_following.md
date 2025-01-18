# Compliant Path Following
This example demonstrates how to simulate a compliant path following control strategy for a robot
moving along a predefined path. One problem with traditional path following controllers is that
the reference moves irrespective of the robot's behaviour. If an obstacle is encountered, the
reference will continue to move, potentially causing the robot to exert large forces on the
obstacle, and inducing large tracking errors.

A compliant path following controller, is driven along the path of the trajectory by a virtual
mechanism system, which is connected to the robot's end effector by a virtual spring and damper.
Thus, if an obstacle is encountered, the connection between the robot and the virtual mechanism
will stop the ``reference'' from moving, and the robot will behave compliantly. Thus compliant
behaviour can be achieved *along the trajectory*, while still stiffly constraining the robot to
the path.

The robot is controlled by a virtual mechanism system, which includes a virtual track (trajectory)
that a virtual cart moves along. A virtual spring and damper are used to connect the virtual cart
to the robot's end effector, and a force source applies a driving force to the virtual cart.

The robot is controlled to follow a predefined path while being subjected to a disturbance force.
The result is a controller which stiffly constrains the robot to the path, while behaving
compliantly along the direction of travel. When a force exceeding the driving force is applied to
the robot, the cart may even reverse its direction of travel, but the linear spring will
ensure that the robot remains on the path.

````julia
using DifferentialEquations
using GLMakie
using LinearAlgebra

using VMRobotControl
using VMRobotControl.Splines: CubicSpline
using StaticArrays
````

## Loading/Building the Robot and Controller

First, we define the path that the robot will follow. The path is defined as a spline, which is
a piecewise cubic polynomial that interpolates a set of points. We construct the spline using
a set of points that define the path, and then create a `CubicSpline` object from these points,
which will pass exactly through the points.

Care must be taken to ensure that the path is smooth and continuous, as the units of the system
state are in meters and radians: if there are large differences in scale between the points, on
the path, then the automatic tolerance selection in the ODE solver may struggle to perform well,
as in some regions of the path a small difference in `q` may cause a large difference in the
position, requiring a small step size to accurately simulate the system.

````julia
start = SVector(0.425, 0.15, 0.10)
s_width = 4e-2
s_length = 20e-2
spline = let w=s_width, x0=start[1], y0=start[2], L=s_length, h=start[3], Nk = 6, Nr = 6
    forward = Vector(LinRange(x0, x0+L, Nk))
    backward = Vector(reverse(forward))
    spline_points = Matrix{Float64}(undef, 0, 3);

    for i = 0:(Nr-1)
        knots = (i%2)!=1 ? forward : backward
        spline_points = vcat(
            spline_points,
            hcat(knots, y0 .+ w*i*ones(size(knots)), h*ones(size(knots)))
        )
    end
    CubicSpline(spline_points)
end
````

````
VMRobotControl.Splines.CubicSpline{3, Float64}(VMRobotControl.Splines.CubicSplineData{3, Float64}([0.425 0.15 0.1; 0.465 0.15 0.1; 0.505 0.15 0.1; 0.545 0.15 0.1; 0.585 0.15 0.1; 0.625 0.15 0.1; 0.625 0.19 0.1; 0.585 0.19 0.1; 0.545 0.19 0.1; 0.505 0.19 0.1; 0.465 0.19 0.1; 0.425 0.19 0.1; 0.425 0.22999999999999998 0.1; 0.465 0.22999999999999998 0.1; 0.505 0.22999999999999998 0.1; 0.545 0.22999999999999998 0.1; 0.585 0.22999999999999998 0.1; 0.625 0.22999999999999998 0.1; 0.625 0.27 0.1; 0.585 0.27 0.1; 0.545 0.27 0.1; 0.505 0.27 0.1; 0.465 0.27 0.1; 0.425 0.27 0.1; 0.425 0.31 0.1; 0.465 0.31 0.1; 0.505 0.31 0.1; 0.545 0.31 0.1; 0.585 0.31 0.1; 0.625 0.31 0.1; 0.625 0.35 0.1; 0.585 0.35 0.1; 0.545 0.35 0.1; 0.505 0.35 0.1; 0.465 0.35 0.1; 0.425 0.35 0.1], [0.425 0.15 0.1; 0.4650404299101982 0.14992992148898976 0.10000000000000002; 0.5048382803592072 0.1502803140440408 0.10000000000000003; 0.545606448652973 0.148948822334847 0.1; 0.582735925028901 0.1539243966165712 0.10000000000000003; 0.6334498512314223 0.1353535911988681 0.1; 0.63346467004541 0.20466123858795626 0.10000000000000002; 0.5826914685869382 0.18600145444930694 0.1; 0.5457694556068371 0.19133294361481612 0.10000000000000002; 0.5042307089857143 0.18866677109142868 0.10000000000000002; 0.46730770845030617 0.19399997201946925 0.1; 0.4165384572130613 0.17533334083069424 0.10000000000000002; 0.4165384626974484 0.24466666465775389 0.1; 0.46730769199714484 0.22600000053829014 0.10000000000000002; 0.5042307693139725 0.23133333318908558 0.1; 0.5457692307469653 0.22866666670536728 0.10000000000000002; 0.582692307698167 0.23399999998944523 0.10000000000000002; 0.6334615384603667 0.21533333333685156 0.10000000000000002; 0.6334615384603667 0.28466666666314844 0.10000000000000002; 0.5826923076981669 0.2660000000105548 0.10000000000000002; 0.5457692307469654 0.27133333329463266 0.10000000000000002; 0.5042307693139725 0.26866666681091445 0.10000000000000002; 0.4673076919971448 0.27399999946170983 0.10000000000000002; 0.4165384626974485 0.2553333353422461 0.10000000000000002; 0.4165384572130613 0.32466665916930576 0.10000000000000002; 0.46730770845030606 0.3060000279805307 0.10000000000000002; 0.5042307089857143 0.31133322890857135 0.10000000000000002; 0.5457694556068371 0.30866705638518394 0.10000000000000002; 0.5826914685869382 0.31399854555069306 0.10000000000000002; 0.6334646700454099 0.29533876141204374 0.10000000000000002; 0.6334498512314224 0.36464640880113186 0.10000000000000002; 0.5827359250289011 0.3460756033834287 0.10000000000000002; 0.545606448652973 0.351051177665153 0.10000000000000002; 0.5048382803592073 0.3497196859559591 0.10000000000000002; 0.4650404299101983 0.35007007851101013 0.10000000000000003; 0.425 0.35 0.1]))
````

Because the meshes for the franka robot are in the DAE file format, which is not natively
supported by MeshIO/FileIO, we have to manually register the DAE file format to be able to load
Register DAE file format to DigitalAssetExchangeFormatIO, so that the mesh files for the franka
robot can be loaded. Most of the time, this is done automatically by the package that provides the
file format, but in this case we have to do it manually, before we can load the URDF file.
Then we load the URDF file, with warnings suppressed, as the URDF file contains some features
that are not supported by the URDFParser, but are not necessary for this example.

We then add gravity compensation to the robot model. As the franka robot does its own gravity
compensation, this is considered part of the robot model, rather than part of the controller, so
it is added directly to `robot`, not to the virtual mechanism system.

We then define a tool center point (TCP), in the end-effector frame of the robot, and add a
coordinate to the robot model to represent the TCP. We also add a coordinate to the robot model
for each joint, and add a tanh damper component to each joint to make the simulation more
realistic, emulating the coulomb friction in the joints. The choice of β will affect the
minimum velocity at which the damper will start to act, and will therefore affect the
number of timesteps required to simulate the system: a smaller β will require more timesteps.

````julia
using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseRSON(joinpath(module_path, "RSONs/rsons/franka_panda/pandaSurgical.rson"))
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
add_coordinate!(robot, FrameOrigin("instrument_EE_frame"); id="TCP")

for (i, τ_coulomb) in zip(1:7, [5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0])
    β = 1e-1
    add_component!(robot, TanhDamper(τ_coulomb, β, "J$i"); id="J$(i)_damper")
end;
````

````
┌ Warning: lines is not supported
└ @ DigitalAssetExchangeFormatIO C:\Users\omaar\.julia\packages\DigitalAssetExchangeFormatIO\joSI4\src\parser.jl:127
┌ Warning: lines is not supported
└ @ DigitalAssetExchangeFormatIO C:\Users\omaar\.julia\packages\DigitalAssetExchangeFormatIO\joSI4\src\parser.jl:127
┌ Warning: lines is not supported
└ @ DigitalAssetExchangeFormatIO C:\Users\omaar\.julia\packages\DigitalAssetExchangeFormatIO\joSI4\src\parser.jl:127

````

Now, we build the virtual mechanism, which consists of a virtual track that a virtual cart moves
along. The virtual track is a `Rail` joint based upon the spline that we defined earlier. The
virtual cart is connected to the robot's end effector by a virtual spring and damper, using the
coordinate `CartPosition`.
Coordinate `CartDistance` is the jointspace coordinate of the rail joint: distance the cart has
moved along the virtual track.
Components `CartInertance` and `CartDamper` are used to add inertance and damping to the cart,
and can be thought of as the mass and friction of the cart, respectively.
Notably, using a `LinearInerter` component here is equivalent to adding a mass to the cart which
is not affected by the force of gravity.

````julia
vm = Mechanism{Float64}("VirtualTrack")

cart_frame = add_frame!(vm, "Cart")
add_joint!(vm, Rail(spline, zero(Transform{Float64}));
        parent=root_frame(vm), child=cart_frame,        id="RailJoint")
````

````
"RailJoint"
````

Jointspace components

````julia
add_coordinate!(vm, JointSubspace("RailJoint");         id="CartDistance")
add_coordinate!(vm, FrameOrigin(cart_frame);            id="CartPosition")
add_component!(vm, LinearInerter(1.0, "CartPosition");  id="CartInertance") # Cart mass
add_component!(vm, LinearDamper(100.0, "CartPosition"); id="CartDamper");
````

Now, we will combine the robot and the virtual mechanism into a single system. To do this
we will create a `VirtualMechanismSystem` object, which will contain both the robot and the
virtual mechanism. We will then add a `CoordDifference` component to the system, which will
take the difference between the position of the virtual cart and the position of the robot's
end effector. We will then add a spring and damper component to the system, which will constrain
the robot to the path defined by the cart.

Finally we will add a `ForceSource` component to the system, which will apply a driving force
to the cart, to move it along the path. The force source will apply a force of 20N in the forward
direction, unless doing so would exceed the maximum power of 10W, in which case the magnitude of
the force will be reduced to ensure that the power does not exceed 10W.

````julia
vms = VirtualMechanismSystem("System", robot, vm)
err_coord = CoordDifference(".virtual_mechanism.CartPosition", ".robot.TCP")
err_spring = LinearSpring(3000.0 * identity(3), "CartError")
err_damper = LinearDamper(50.0 * identity(3), "CartError")

add_coordinate!(vms, err_coord;     id="CartError")
add_component!(vms, err_spring;     id="CartErrSpring")
add_component!(vms, err_damper;     id="CartErrDamper")

max_power = 10.0
force_source = ForceSource(SVector(20.0), max_power, ".virtual_mechanism.CartDistance")
add_component!(vms, force_source;   id="Force source");
````

## Setting up the simulation
We define a disturbance function that will apply a force of 20N in the negative x-direction
between 3 and 6 seconds. We then define the `f_control` function, which will apply this
disturbance force to the system. The `f_control` function is called once per timestep of the
simulation, and can be used to apply external forces to the system. The `f_setup` function is
used to get the coordinate ID of the TCP position at the beginning of the simulation, to be used
in the `f_control` function.

We then define the timespan, initial joint angles, joint velocities, and gravity vector for the
simulation. We create a dynamics cache, and an ODE problem, and solve the ODE problem using the
Tsit5 solver from DifferentialEquations.jl.

````julia
f_setup(cache) = get_compiled_coordID(cache, ".robot.TCP")
disturbance_func(t) = 3 < t < 6 ? SVector(-20., 0., 0.) : SVector(0., 0., 0.)
function f_control(cache, t, args, extra)
    tcp_pos_coord_id = args
    F = disturbance_func(t)
    uᵣ, uᵥ = get_u(cache)
    z = configuration(cache, tcp_pos_coord_id)
    J = jacobian(cache, tcp_pos_coord_id)
    mul!(uᵣ, J', F)
    nothing
end

cache = new_dynamics_cache(compile(vms))
tspan = (0.0, 10.0)
q = ([0.0, 0.1, 0.0, -2.5, pi/2, pi/2, 0.6], [0.0])
q̇ = (zeros(7), zeros(1))
gravity = VMRobotControl.DEFAULT_GRAVITY
prob = get_ode_problem(cache, gravity, q, q̇, tspan; f_setup, f_control)
@info "Simulating compliant path following."
sol = solve(prob, Tsit5(); maxiters=2e5, abstol=1e-6, reltol=1e-6);
````

````
[ Info: Simulating compliant path following.

````

## Visualizing the simulation
We will now visualize the simulation. We will create a `Figure` object, and add a `LScene` to it.
We create an observable for the time, and an observable for the kinematics cache. These will
be used by the `animate_robot_odesolution` function, which will update the observables, and
therefore any plots that depend upon them, as it plays back the simulation solution.

````julia
plotting_t = Observable(0.0)
plotting_kcache = Observable(new_kinematics_cache(compile(vms)))
plotting_vm_kcache = map(plotting_kcache) do k
    VMRobotControl.virtual_mechanism_cache(k)
end
cartID = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.CartPosition")

fig = Figure(; size=(720, 720), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
cam = cam3d!(ls; center=false)
cam.eyeposition[] = [0.912748151284803, 1.0895512157983234, 0.8513286633206905]
cam.lookat[] = [0.09731484912188403, -0.18195162102725565, 0.17343471031108892];
````

The cart is visualized as a red rectangle. The rail is shown by sketching the virtual mechanism,
and the robot is visualized using the robotvisualize! function.

````julia
scatter!(ls, plotting_kcache, cartID; color=:red, marker=:rect, markersize=10)
robotvisualize!(ls, plotting_kcache)
robotsketch!(ls, plotting_vm_kcache);
````

To plot the force arrow, we first get the TCP position, and then calculate the force at the TCP
position using the disturbance function. We then plot the force arrow at the TCP position.
The size of the arrow is scaled according to the magnitude of the force.

````julia
tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.TCP")
tcp_pos = map(plotting_kcache) do kcache
    Point3f(configuration(kcache, tcp_pos_id))
end
force = map(t -> 0.01 * Vec3f(disturbance_func(t)), plotting_t)
arrowsize = map(f -> 0.1*(f'*f)^(0.25), force)
arrows!(ls, map(p -> [p], tcp_pos), map(f -> [f], force); color = :red, arrowsize)

fps = 60
T = sol.t[end]
N_frames = Int(floor(fps * T))
ts = LinRange(0.0, T, N_frames)
savepath = joinpath(module_path, "docs/src/assets/compliant_path_following.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t);
````

```@raw html
<video controls width="100%" height="auto" autoplay loop>
<source src="../../assets/compliant_path_following.mp4" type="video/mp4">
</video>
```

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

