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
using JLD2
using VMRobotControl:
    DEFAULT_GRAVITY,
    DEFAULT_F_SETUP,
    DEFAULT_F_CONTROL,
    robot_ndof,
    remake

ROSPY_LISTEN_PORT = 25342

SAVE_DATA = false

const START = "START"
const WARMUP_DONE = "WARMUP_DONE"
const STOP = "STOP"
const STOP_ACK = "STOP_ACK"

mutable struct RobotStatePacket
    timestamp::UInt64
    sequence_number::UInt64
    state::Vector{Float64}
end

mutable struct TargetPosPacket
    target_pos::Vector{Float64}
end

mutable struct ROSPyConnectionStatus
    sequence_number::UInt64
    received_packets::UInt64
    processed_packets::UInt64
    last_received::RobotStatePacket
    target_pos_received::TargetPosPacket
end

function print_connection_status_summary(connection)
    (;status_lock, status) = connection
    Base.@lock status_lock begin
        percent_received = 100 * status.received_packets / status.sequence_number
        percent_processed = 100 * status.processed_packets / status.sequence_number
        missed = status.sequence_number - status.processed_packets
        # @printf "SN: %i Received: %.3f%% Processed %.3f%% Missed: %i           \r" status.sequence_number percent_received percent_processed missed
    end
end

struct ROSPyClientConnection
    # config
    num_torques::Int
    num_states::Int
    num_targets::Int
    # Sockets
    command_socket::TCPSocket
    data_socket::UDPSocket
    # State
    status::ROSPyConnectionStatus
    # Async tasks/buffers for sending/receiving data on UPD/TCP sockets
    stop::Threads.Atomic{Bool}
    recv_tcp_task::Task
    recv_tcp_lock::ReentrantLock
    tcp_buffer::IOBuffer
    recv_udp_task::Task
    status_lock::ReentrantLock
    torques::Vector{Float64}
    send_buffer::Vector{UInt8}
    function ROSPyClientConnection(
            command_socket::TCPSocket,
            num_torques::Int,
            num_states::Int,
            num_targets::Int
        )
        # Bind the data socket to the same address as the command socket
        @assert isopen(command_socket)
        bound_ip, bound_port = getsockname(command_socket) 
        data_socket = UDPSocket()
        bind(data_socket, bound_ip, bound_port)
        # Each "target" is a 3D position (x,y,z)
        status = ROSPyConnectionStatus(0, 0, 0, RobotStatePacket(0, 0, zeros(num_states)), TargetPosPacket(zeros(3*num_targets)))
        # Start a task to receive commands on the tcp socket
        stop = Threads.Atomic{Bool}(false)
        recv_tcp_lock = ReentrantLock()
        tcp_buffer = PipeBuffer()
        recv_tcp_task = @async begin
            while !eof(command_socket) && !stop[]
                data = readline(command_socket)
                isempty(data) && continue
                lock(recv_tcp_lock) do
                    write(tcp_buffer, data)
                    write(tcp_buffer, '\n')
                end
                yield()
            end
            nothing
        end
        # Start a task to receive data on the data socket
        status_lock = ReentrantLock()
        recv_udp_task = @async begin
            while !stop[]
                data = recv(data_socket)
                parse_rospy_packet!(status_lock, status, data)
                yield()
            end
        end
        # Allocate space for sending data from
        torques = zeros(num_torques)
        send_buffer = Vector{UInt8}(undef, sizeof(Int) + sizeof(Float64) + num_torques * sizeof(Float64))
        new(
            num_torques,
            num_states,
            num_targets, # Not sure what exactly is this
            command_socket,
            data_socket,
            status,
            stop,
            recv_tcp_task,
            recv_tcp_lock,
            tcp_buffer,
            recv_udp_task,
            status_lock,
            torques,
            send_buffer
        )
    end
end

function my_time_ns()
    UInt64(time() * 1e9)
end

function parse_rospy_packet!(status_lock, status::ROSPyConnectionStatus, data::Vector{UInt8})
    Base.@lock status_lock begin
        expected_length = sizeof(UInt64) + sizeof(UInt64) +
         length(status.last_received.state) * sizeof(Float64) + length(status.target_pos_received.target_pos) * sizeof(Float64)
        if length(data) != expected_length
            error("Invalid rospy data packet length: expected $expected_length, got $(length(data))")
        end
        b = IOBuffer(data; read=true, write=false)
        status.last_received.timestamp = ntoh(read(b, UInt64))
        status.last_received.sequence_number = ntoh(read(b, UInt64))
        for i in eachindex(status.last_received.state)
            status.last_received.state[i] = ntoh(read(b, Float64))
        end
        for j in eachindex(status.target_pos_received.target_pos)
            status.target_pos_received.target_pos[j] = ntoh(read(b, Float64))
        end
        status.received_packets += 1
    end
    nothing
end

function _connect(rospy_ip, rospy_port, num_torques::Int, num_states::Int, num_targets::Int)
    command_socket = connect(rospy_ip, rospy_port)
    ROSPyClientConnection(command_socket, num_torques, num_states, num_targets)
end

function _cleanup!(connection::ROSPyClientConnection)
    @info "Closing connection"
    write(connection.command_socket, "$STOP\n")
    # This will gracefully stop the recv tasks unless they are blocked
    connection.stop[] = true 
    sleep(0.1)
    # Closing the data socket will cause the recv_udp_task to throw an error
    close(connection.data_socket) # This causes an error 
    try
        fetch(connection.recv_udp_task)
    catch EOFError
    end
    # Closing the command socket will cause the recv_tcp_task to exit
    close(connection.command_socket)
    fetch(connection.recv_tcp_task)
    connection
end

function with_rospy_connection(f::Function, rospy_ip, rospy_port, num_torques::Int, num_states::Int, num_targets::Int)
    connection = _connect(rospy_ip, rospy_port, num_torques, num_states, num_targets)
    try
        f(connection)
    finally
        if SAVE_DATA
            @save "examples/07April_1.jld2" time_vector LeftFingerTarget_intended RightFingerTarget_intended BaseTarget_intended LeftFingerTarget_actual RightFingerTarget_actual BaseTarget_actual LeftFinger_intended RightFinger_intended Base_intended LeftFinger_actual RightFinger_actual Base_actual RepulsivePosition_right RepulsivePosition_left RepulsivePosition_front RightDamper_velocity LeftDamper_velocity BaseDamper_velocity HandPositions
        end
        _cleanup!(connection)
    end
    connection
end

function check_tcp(connection::ROSPyClientConnection)::Union{Nothing, String}
    if istaskfailed(connection.recv_tcp_task)
        fetch(connection.recv_tcp_task) # Re-throw the error
        error()
    elseif istaskdone(connection.recv_tcp_task)
        error("TCP connection/command socket closed")
    end
    yield()
    line = Base.@lock connection.recv_tcp_lock begin
        readline(connection.tcp_buffer)
    end
    if isempty(line)
        return nothing
    else
        @debug "Received command over TCP: \"$line\""
        return line
    end
end

function _get_new_packet_to_process!(connection::ROSPyClientConnection, packet::RobotStatePacket)
    @assert packet.sequence_number > connection.status.sequence_number
    connection.status.sequence_number = packet.sequence_number
    connection.status.processed_packets += 1
    (deepcopy(connection.status.target_pos_received), deepcopy(connection.status.last_received))
end

function check_udp(connection::ROSPyClientConnection)
    if istaskfailed(connection.recv_udp_task)
        fetch(connection.recv_udp_task) # Re-throw the error
        error()
    elseif istaskdone(connection.recv_udp_task)
        error("UDP data socket closed")
    end
    
    # Then check if we received a packet on the data socket
    Base.@lock connection.status_lock begin
    #print(connection.status.last_received.timestamp)
        if connection.status.last_received.sequence_number > connection.status.sequence_number
            return _get_new_packet_to_process!(connection, connection.status.last_received)
        else
            # @debug "No new UDP message to process"
            return (nothing, nothing)
        end
    end
end

function send_torques(connection::ROSPyClientConnection, sequence_number::UInt64)
    # Send the send buffer on the data socket
    @assert length(connection.send_buffer) == sizeof(Int) + sizeof(Float64) + connection.num_torques * sizeof(Float64)
    b = IOBuffer(connection.send_buffer; read=false, write=true)
    
    write(b, hton(sequence_number))
    write(b, hton(my_time_ns()))
    for torque in connection.torques
        write(b, hton(torque))
    end
    remote_ip, remote_port = getpeername(connection.command_socket)
    data = take!(b)
    @assert length(data) == sizeof(UInt) + sizeof(UInt) + connection.num_torques * sizeof(Float64)
    # @debug "Sending $(length(data)) bytes to $remote_ip:$remote_port"
    send(connection.data_socket, remote_ip, remote_port, data)
end


const STATE_WARMUP = 1
const STATE_ACTIVE = 2
const STATE_STOPPED = 3

function loop_warmup(connection::ROSPyClientConnection, control_func!::Function)
    @info "State: WARMUP"
    start_time = time()
    while true
        yield() # To allow for interrupts
        command = check_tcp(connection)::Union{String, Nothing}
        if isnothing(command)
            data_targets, data = check_udp(connection)
            if !isnothing(data)
                i, t, dt = 0, 0.0, 0.0
                stop = control_func!(connection.torques, data.state, i, data_targets.target_pos, t, dt)
                stop && return STATE_STOPPED # Stop the controller gracefully
                any(isnan, connection.torques) && error("Control function returned NaN torques: $(connection.torques)")
                send_torques(connection, data.sequence_number)
                print_connection_status_summary(connection)
            end
        else
            error("Received unexpected command during warmup: \"$command\"")
        end
        if time() - start_time > 1.0
            write(connection.command_socket, "$WARMUP_DONE\n")
            return STATE_ACTIVE
        end   
    end
end

function loop_active(connection::ROSPyClientConnection, control_func!::Function)
    @info "State: ACTIVE"
    i = 1
    t = 0.0
    while true
        yield() # To allow for interrupts
        command = check_tcp(connection)::Union{String, Nothing}
        if isnothing(command)
            data_targets, data = check_udp(connection)
            if !isnothing(data)
                iswarmup = true
                i, dt, t = let t0 = t; t = time(); (i+1, t - t0, t) end
                stop = control_func!(connection.torques, data.state, i, data_targets.target_pos, t, dt)
                stop && return STATE_STOPPED # Stop the controller gracefully
                any(isnan, connection.torques) && error("Control function returned NaN torques: $(connection.torques)")
                send_torques(connection, data.sequence_number)

                print_connection_status_summary(connection)
            end
        else
            error("Received unexpected command while active: \"$command\"")
        end
    end
end

function warmup_and_activate(connection::ROSPyClientConnection, control_func!::Function; )
    if connection.stop[] != false
        error("Connection already used, create a new connection instead.")
    end

    @info "Sending $START"
    write(connection.command_socket, "$START\n")

    state = STATE_WARMUP
    while true
        if state == STATE_WARMUP
            state = loop_warmup(connection, control_func!)
        elseif state == STATE_ACTIVE
            state = loop_active(connection, control_func!)
        elseif state == STATE_STOPPED
            break
        else
            error("Invalid state: $state")
        end
    end
end

function get_initial_state(connection; retries=100, sleep_time=0.2, max_initial_state_age)
    for i = 1:retries
        Base.@lock connection.status_lock begin
            if connection.status.last_received.sequence_number > 0
                initial_state = deepcopy(connection.status.last_received)
                targets_state = deepcopy(connection.status.target_pos_received)
                t_ns = my_time_ns()           
                if (t_ns - initial_state.timestamp)/1e9 > max_initial_state_age 
                    println("Initial state is too old, $((t_ns - initial_state.timestamp)/1e9)... retrying")
                    # Ensure the state is recent
                else
                    return (targets_state, initial_state)
                end
            end
        end
        i < retries && sleep(sleep_time)
    end
    error("Failed to get initial state after $retries retries")
end

function ros_vm_controller(
        connection::ROSPyClientConnection,
        vms,
        qᵛ; 
        gravity=DEFAULT_GRAVITY,
        f_setup=DEFAULT_F_SETUP,
        f_control=DEFAULT_F_CONTROL,
        E_max=1.0,
        max_initial_state_age=1.0
    )
    # Create cache
    control_cache = new_control_cache(vms, qᵛ, gravity)
    let # Set initial joint state in cache
        target_positions, state = get_initial_state(connection; max_initial_state_age)
        NDOF = robot_ndof(control_cache)
        qʳ = view(state.state, 1:NDOF)
        q̇ʳ = zeros(eltype(control_cache), NDOF)
        target_positions_ = view(target_positions.target_pos, 1:9)
        control_step!(control_cache, 0.0, qʳ, q̇ʳ) # Step at t=0 to set initial state
    end
    
    args = f_setup(control_cache) # Call user setup function

    # Create control callback
    control_func! = let control_cache=control_cache, args=args
        function control_func!(torques, state, i, target_positions, t, dt)
            NDOF = robot_ndof(control_cache)
            @assert length(state) == 2*NDOF
            qʳ = view(state, 1:NDOF)
            q̇ʳ = view(state, NDOF+1:2*NDOF)
            target_positions_ = view(target_positions, 1:length(target_positions)) # 3 targets with 3 dimensions each (x,y,z)
            # Main control step
            f_control(control_cache, target_positions_, t, args, (dt, i)) # Call user control function
            torques .= control_step!(control_cache, t, qʳ, q̇ʳ) # Get torques
            return false
        end
    end
    # Check that stored energy is within bounds
    (E = stored_energy(control_cache)) > E_max && error("Initial stored energy exceeds $(E_max)J, was $(E)J")
    warmup_and_activate(connection, control_func!; )
end


# function add_bounded_region_tanh_spring!(mech, stiffness_input, max_force_input, bounds, coord_id)
#     lb, ub = bounds
#     add_coordinate!(mech, ConstCoord(SVector(lb, lb, lb)); id="lb_$coord_id") # < needs unique name? 
#     add_coordinate!(mech, ConstCoord(SVector(lb, lb, lb)); id="ub_$coord_id")
#     add_coordinate!(mech, CoordDifference(coord_id, "lb_$coord_id"); id="ext_lower_$coord_id")
#     add_coordinate!(mech, CoordDifference(coord_id, "ub_$coord_id"); id="ext_upper_$coord_id")

#     center_spring = TanhSpring(coord_id; max_force=max_force_input, stiffness=stiffness_input)
#     lower_spring = TanhSpring("ext_lower_$coord_id"; max_force=max_force_input/2, stiffness=150.0)
#     upper_spring = TanhSpring("ext_upper_$coord_id"; max_force=max_force_input/2, stiffness=150.0)

#     add_component!(mech, center_spring; id="$(coord_id)_spring_center")
#     add_component!(mech, lower_spring; id="$(coord_id)_spring_lower")
#     add_component!(mech, upper_spring; id="$(coord_id)_spring_upper")
# end

# function add_plane_region_spring!(mechanism, stiffness, bound, coord_id)
#     lb, ub = bounds
#     lb_coord_id = add_coordinate!(mechanism, ConstCoord(SVector(lb, lb, lb)); id="lb_$coord_id")
#     ub_coord_id = add_coordinate!(mechanism, ConstCoord(SVector(ub, ub, ub)); id="ub_$coord_id")
#     ext_lower = add_coordinate!(mechanism, CoordDifference(coord_id, lb_coord_id); id="ext_lower_$coord_id")
#     ext_upper = add_coordinate!(mechanism, CoordDifference(coord_id, ub_coord_id); id="ext_upper_$coord_id") 
#     s_lower = ReLUSpring(stiffness, ext_lower, true) # true ⟹ Flip direction of rectification
#     s_upper = ReLUSpring(stiffness, ext_upper, false) 
#     add_component!(mechanism, s_lower; id="$(coord_id)_spring_lower")
#     add_component!(mechanism, s_upper; id="$(coord_id)_spring_upper")
#     mechanism
# end

#### Now load the robot and the VMC ###

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
    add_deadzone_springs!(robot, 100.0, (limits.lower+0.3, limits.upper-0.3), "J$i")
    if i == 7
        add_component!(robot, TanhDamper(7.0, 0.2, "J$i");         id="JointDamper$i")
        println("added damper for joint 7")
    else
        add_component!(robot, TanhDamper(τ_coulomb, β, "J$i");         id="JointDamper$i")
    end
end;


add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., 0.04, 0.0)); id="RealLeftFinger")
add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., -0.04, 0.0)); id="RealRightFinger")
add_coordinate!(robot, FrameOrigin("panda_hand"); id="RealHandBase")
add_coordinate!(robot, FrameOrigin("panda_hand_tcp"); id="MiddlePointGripper")

points_distance_gripper = 0.9
points_distance_TCP = 0.5
add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., points_distance_gripper/2, 0.0)); id="LeftFinger")
add_coordinate!(robot, FramePoint("panda_hand_tcp", SVector(0., -points_distance_gripper/2, 0.0)); id="RightFinger")
add_coordinate!(robot, FramePoint("panda_hand", SVector(0., 0., -(points_distance_TCP - 0.1034))); id="HandBase") # 0.1034: distance between gripper and TCP
# add_coordinate!(robot, FrameOrigin("panda_hand"); id="pandaHand")

vms = VirtualMechanismSystem("RobotHandover", robot)
vm = vms.virtual_mechanism

repulsive_fields = Dict(
    "repulsiveField1" => SVector(0.5,  0.1, 0.3),
    "repulsiveField2" => SVector(0.3, -0.2, 0.1),
    "repulsiveField3" => SVector(0.3, -0.2, 0.1)
)

graspingPoints = String[]
# graspingPoints = ["RealLeftFinger", "RealRightFinger", "RealHandBase"]
graspingPoints = ["MiddlePointGripper", "RealHandBase"]

for (id, pos) in repulsive_fields
    add_coordinate!(vm, ReferenceCoord(Ref(pos)); id)
end

for id in graspingPoints
    for repulsive_field in keys(repulsive_fields)
        add_coordinate!(vms, CoordDifference(".robot.$id", ".virtual_mechanism.$repulsive_field"); id="$repulsive_field $id error")
        add_component!(vms, GaussianSpring("$repulsive_field $id error"; max_force=-15.0, width=0.05); id="$repulsive_field $id spring")
    end
end

add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.95, 0.5))); id="LeftFingerTarget")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.05, 0.5))); id="RightFingerTarget")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.46, 0.5))); id="RealRightFingerTarget")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.54, 0.5))); id="RealLeftFingerTarget")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.5, 1.0))); id="HandBaseTarget")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.3, -0.5, 0.6034))); id="RealHandBaseTarget")

add_coordinate!(vms, CoordDifference(".robot.LeftFinger", ".virtual_mechanism.LeftFingerTarget"); id="L pos error")
add_coordinate!(vms, CoordDifference(".robot.RightFinger", ".virtual_mechanism.RightFingerTarget"); id="R pos error")
add_coordinate!(vms, CoordDifference(".robot.HandBase", ".virtual_mechanism.HandBaseTarget"); id="H pos error")
# add_coordinate!(vms, CoordDifference(".robot.RealRightFinger", ".virtual_mechanism.RealRightFingerTarget"); id="RR pos error")
# add_coordinate!(vms, CoordDifference(".robot.RealLeftFinger", ".virtual_mechanism.RealLeftFingerTarget"); id="RL pos error")
add_coordinate!(vms, CoordDifference(".robot.RealHandBase", ".virtual_mechanism.RealHandBaseTarget"); id="RH pos error")
add_coordinate!(vms, CoordDifference(".robot.RealRightFinger", ".virtual_mechanism.RealRightFingerTarget"); id="RR pos error")

# add_bounded_region_tanh_spring!(vms, 250.0, 3.0, (-0.5, 0.5), "L pos error")
# add_bounded_region_tanh_spring!(vms, 250.0, 3.0, (-0.5, 0.5), "R pos error")
# add_bounded_region_tanh_spring!(vms, 500.0, 3.0, (-0.5, 0.5), "H pos error")
default_stiffness = 10.0
additional_stiffness = 100.0
default_max_force = 10.0
default_damping = 3.0
max_force_small_range = 5.0
add_component!(vms, TanhSpring("L pos error"; max_force=default_max_force, stiffness=default_stiffness); id="L spring")
add_component!(vms, LinearDamper(default_damping, "L pos error"); id="L damper")
add_component!(vms, TanhSpring("R pos error"; max_force=default_max_force, stiffness=default_stiffness); id="R spring")
add_component!(vms, LinearDamper(default_damping, "R pos error"); id="R damper")
# add_component!(vms, TanhSpring("RR pos error"; max_force=default_max_force, stiffness=default_stiffness); id="RR spring")
# add_component!(vms, LinearDamper(default_damping, "RR pos error"); id="RR damper")
# add_component!(vms, TanhSpring("RL pos error"; max_force=default_max_force, stiffness=default_stiffness); id="RL spring")
# add_component!(vms, LinearDamper(default_damping, "RL pos error"); id="RL damper")
add_component!(vms, TanhSpring("H pos error"; max_force=default_max_force, stiffness=default_stiffness); id="H spring")
add_component!(vms, LinearDamper(default_damping, "H pos error"); id="H damper")
# add_component!(vms, TanhSpring("RH pos error"; max_force=default_max_force, stiffness=default_stiffness); id="RH spring")
# add_component!(vms, LinearDamper(default_damping, "RH pos error"); id="RH damper")
add_component!(vms, TanhSpring("H pos error"; max_force=max_force_small_range, stiffness=additional_stiffness); id="AH spring")
# add_component!(vms, TanhSpring("RR pos error"; max_force=max_force_small_range, stiffness=500.0); id="ARR spring")
# add_component!(vms, TanhSpring("RL pos error"; max_force=max_force_small_range, stiffness=500.0); id="ARL spring")
add_component!(vms, TanhSpring("R pos error"; max_force=max_force_small_range, stiffness=additional_stiffness); id="AR spring")
add_component!(vms, TanhSpring("L pos error"; max_force=max_force_small_range, stiffness=additional_stiffness); id="AL spring")


# add_bounded_region_tanh_spring!(vms, default_stiffness, default_max_force, (-0.1, 0.1), "L pos error")
# add_component!(vms, LinearDamper(default_damping, "L pos error"); id="L damper")
# add_bounded_region_tanh_spring!(vms, default_stiffness, default_max_force, (-0.1, 0.1), "R pos error")
# add_component!(vms, LinearDamper(default_damping, "R pos error"); id="R damper")
# add_bounded_region_tanh_spring!(vms, default_stiffness, default_max_force, (-0.1, 0.1), "RR pos error")
# add_component!(vms, LinearDamper(default_damping, "RR pos error"); id="RR damper")
# add_bounded_region_tanh_spring!(vms, default_stiffness, default_max_force, (-0.1, 0.1), "H pos error")
# add_component!(vms, LinearDamper(default_damping, "H pos error"); id="H damper")

# K = SMatrix{3, 3}(100., 0., 0., 0., 100., 0., 0., 0., 100.)
# add_component!(vms, LinearSpring(K, "R pos error");       id="R spring")
# add_component!(vms, LinearSpring(K, "L pos error");       id="L spring")
# add_component!(vms, LinearSpring(K, "H pos error");       id="H spring")

time_vector = Float64[]

LeftFingerTarget_intended = SVector{3, Float64}[] # These are the "target" points connected with springs and dampers
RightFingerTarget_intended = SVector{3, Float64}[]
BaseTarget_intended = SVector{3, Float64}[]

LeftFingerTarget_actual = SVector{3, Float64}[] # These are the points attached on the robot frame
RightFingerTarget_actual = SVector{3, Float64}[]
BaseTarget_actual = SVector{3, Float64}[]

LeftFinger_intended = SVector{3, Float64}[] # These are the positions the gripper is expected to reach based on the VMC, from Meta Quest
RightFinger_intended = SVector{3, Float64}[]
Base_intended = SVector{3, Float64}[]

LeftFinger_actual = SVector{3, Float64}[] # These are the readings of the current positions of the gripper (similar to Target_actual) 
RightFinger_actual = SVector{3, Float64}[]
Base_actual = SVector{3, Float64}[]

RepulsivePosition_right = SVector{3, Float64}[] # Positions of the virtual field
RepulsivePosition_left = SVector{3, Float64}[]
RepulsivePosition_front = SVector{3, Float64}[]

RightDamper_velocity = SVector{3, Float64}[]
LeftDamper_velocity = SVector{3, Float64}[]
BaseDamper_velocity = SVector{3, Float64}[]

HandPositions = SVector{3, Float64}[]

function f_setup(cache)
    LeftFingerTarget_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.LeftFingerTarget")
    RightFingerTarget_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.RightFingerTarget")
    BaseTarget_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.HandBaseTarget")
    
    repulsiveField1_id = get_compiled_coordID(cache, ".virtual_mechanism.repulsiveField1")
    repulsiveField2_id = get_compiled_coordID(cache, ".virtual_mechanism.repulsiveField2")
    repulsiveField3_id = get_compiled_coordID(cache, ".virtual_mechanism.repulsiveField3")

    leftSpring_id = get_compiled_componentID(cache, "L spring")
    rightSpring_id = get_compiled_componentID(cache, "R spring")
    baseSpring_id = get_compiled_componentID(cache, "H spring")

    leftDamper_coordDiff_id = get_compiled_coordID(cache, "L pos error")
    rightDamper_coordDiff_id = get_compiled_coordID(cache, "R pos error")
    baseDamper_coordDiff_id = get_compiled_coordID(cache, "H pos error")

    positionDiff_id = get_compiled_coordID(cache, "RH pos error")

    damper_R_id = get_compiled_componentID(cache, "R damper")
    damper_L_id = get_compiled_componentID(cache, "L damper")
    damper_H_id = get_compiled_componentID(cache, "H damper")

    LeftFingerRobotFurther_coord_id = get_compiled_coordID(cache, ".robot.LeftFinger")
    RightFingerRobotFurther_coord_id = get_compiled_coordID(cache, ".robot.RightFinger")
    BaseRobotFurther_coord_id = get_compiled_coordID(cache, ".robot.HandBase")

    RealRightFingerRobot_coord_id = get_compiled_coordID(cache, ".robot.RealLeftFinger")
    RealLeftFingerRobot_coord_id = get_compiled_coordID(cache, ".robot.RealRightFinger")
    RealBaseRobot_coord_id = get_compiled_coordID(cache, ".robot.RealHandBase")

    additionalLeftSpring_id = get_compiled_componentID(cache, "AL spring")
    additionalRightSpring_id = get_compiled_componentID(cache, "AR spring")
    additionalBaseSpring_id = get_compiled_componentID(cache, "AH spring")

    return (LeftFingerTarget_coord_id, RightFingerTarget_coord_id, BaseTarget_coord_id, 
    LeftFingerRobotFurther_coord_id, RightFingerRobotFurther_coord_id, BaseRobotFurther_coord_id,
    RealRightFingerRobot_coord_id, RealLeftFingerRobot_coord_id, RealBaseRobot_coord_id,
    repulsiveField1_id, repulsiveField2_id, repulsiveField3_id,
    leftSpring_id, rightSpring_id, baseSpring_id, additionalLeftSpring_id, additionalRightSpring_id, additionalBaseSpring_id,
    positionDiff_id, damper_R_id, damper_L_id, damper_H_id,
    leftDamper_coordDiff_id, rightDamper_coordDiff_id, baseDamper_coordDiff_id )
end

function f_control(cache, target_positions, t, setup_ret, extra)
    LeftFingerTarget_coord_id, RightFingerTarget_coord_id, BaseTarget_coord_id, 
    LeftFingerRobotFurther_coord_id, RightFingerRobotFurther_coord_id, BaseRobotFurther_coord_id,
    RealRightFingerRobot_coord_id, RealLeftFingerRobot_coord_id, RealBaseRobot_coord_id,
    repulsiveField1_id, repulsiveField2_id, repulsiveField3_id,
    leftSpring_id, rightSpring_id, baseSpring_id, additionalLeftSpring_id, additionalRightSpring_id, additionalBaseSpring_id,
    positionDiff_id, damper_R_id, damper_L_id, damper_H_id,
    leftDamper_coordDiff_id, rightDamper_coordDiff_id, baseDamper_coordDiff_id = setup_ret

    LeftFingerTargetPos = SVector(target_positions[1], target_positions[2], target_positions[3])
    cache[LeftFingerTarget_coord_id].coord_data.val[] = LeftFingerTargetPos
    push!(LeftFingerTarget_intended, LeftFingerTargetPos)
    
    RightFingerTargetPos = SVector(target_positions[4], target_positions[5], target_positions[6])
    cache[RightFingerTarget_coord_id].coord_data.val[] = RightFingerTargetPos
    push!(RightFingerTarget_intended, RightFingerTargetPos)

    BaseTargetPos = SVector(target_positions[7], target_positions[8], target_positions[9])
    cache[BaseTarget_coord_id].coord_data.val[] = BaseTargetPos
    push!(BaseTarget_intended, BaseTargetPos)

    LeftFingerFurtherRobot = configuration(cache, LeftFingerRobotFurther_coord_id)
    push!(LeftFingerTarget_actual, LeftFingerFurtherRobot)
    RightFingerFurtherRobot = configuration(cache, RightFingerRobotFurther_coord_id)
    push!(RightFingerTarget_actual, RightFingerFurtherRobot)
    BaseFurtherRobot = configuration(cache, BaseRobotFurther_coord_id)
    push!(BaseTarget_actual, BaseFurtherRobot)

    LeftFingerRobot = configuration(cache, RealLeftFingerRobot_coord_id)
    push!(LeftFinger_actual, LeftFingerRobot)
    RightFingerRobot = configuration(cache, RealRightFingerRobot_coord_id)
    push!(RightFinger_actual, RightFingerRobot)
    BaseRobot = configuration(cache, RealBaseRobot_coord_id)
    push!(Base_actual, BaseRobot)

    LeftFingerFromQuest = SVector(target_positions[10], target_positions[11], target_positions[12])
    push!(LeftFinger_intended, LeftFingerFromQuest)
    RightFingerFromQuest = SVector(target_positions[13], target_positions[14], target_positions[15])
    push!(RightFinger_intended, RightFingerFromQuest)
    BaseFromQuest= SVector(target_positions[16], target_positions[17], target_positions[18])
    push!(Base_intended, BaseFromQuest)

    repulsiveField1Pos = SVector(target_positions[19], target_positions[20], target_positions[21])
    cache[repulsiveField1_id].coord_data.val[] = repulsiveField1Pos
    push!(RepulsivePosition_left, repulsiveField1Pos)

    repulsiveField2Pos = SVector(target_positions[22], target_positions[23], target_positions[24])
    cache[repulsiveField2_id].coord_data.val[] = repulsiveField2Pos
    push!(RepulsivePosition_right, repulsiveField2Pos)

    repulsiveField3Pos = SVector(target_positions[25], target_positions[26], target_positions[27])    
    cache[repulsiveField3_id].coord_data.val[] = repulsiveField3Pos
    push!(RepulsivePosition_front, repulsiveField3Pos)

    leftDamperVelocity = velocity(cache, leftDamper_coordDiff_id)
    push!(LeftDamper_velocity, leftDamperVelocity)

    rightDamperVelocity = velocity(cache, rightDamper_coordDiff_id)
    push!(RightDamper_velocity, leftDamperVelocity)

    baseDamperVelocity = velocity(cache, baseDamper_coordDiff_id)
    push!(BaseDamper_velocity, baseDamperVelocity)

    handPosition = SVector(target_positions[28], target_positions[29], target_positions[30])
    push!(HandPositions, handPosition)

    currentTime = time()
    push!(time_vector, currentTime)

    # damping_val = max(norm(RightFingerRobot - RightFingerFromQuest), norm(BaseRobot - BaseFromQuest))
    damping_val = max(norm(RightFingerTargetPos - RightFingerFurtherRobot), norm(BaseTargetPos - BaseFurtherRobot))
    
    # new_damping = 7.0*tanh(500*damping_val) + 2.0*tanh(10*(damping_val-0.2)) + 2.0*tanh(10*(damping_val+0.2))
    new_damping = 2.0 + 7.0*tanh(10*damping_val)
    # new_damping = 2.0*tanh(20*damping_val)

    if norm(handPosition) > 0.8
        cache[leftSpring_id] = remake(cache[leftSpring_id]; stiffness=0.001)
        cache[rightSpring_id] = remake(cache[rightSpring_id]; stiffness=0.001)
        cache[baseSpring_id] = remake(cache[baseSpring_id]; stiffness=0.001)
        cache[additionalLeftSpring_id] = remake(cache[additionalLeftSpring_id]; stiffness=0.001)
        cache[additionalRightSpring_id] = remake(cache[additionalRightSpring_id]; stiffness=0.001)
        cache[additionalBaseSpring_id] = remake(cache[additionalBaseSpring_id]; stiffness=0.001)
    else
        cache[leftSpring_id] = remake(cache[leftSpring_id]; stiffness=default_stiffness)
        cache[rightSpring_id] = remake(cache[rightSpring_id]; stiffness=default_stiffness)
        cache[baseSpring_id] = remake(cache[baseSpring_id]; stiffness=default_stiffness)
        cache[additionalLeftSpring_id] = remake(cache[additionalLeftSpring_id]; stiffness=additional_stiffness)
        cache[additionalRightSpring_id] = remake(cache[additionalRightSpring_id]; stiffness=additional_stiffness)
        cache[additionalBaseSpring_id] = remake(cache[additionalBaseSpring_id]; stiffness=additional_stiffness)

        cache[damper_R_id] = remake(cache[damper_R_id]; damping=new_damping)
        cache[damper_L_id] = remake(cache[damper_L_id]; damping=new_damping)
        cache[damper_H_id] = remake(cache[damper_H_id]; damping=new_damping)
    end

    nothing 
end

# function f_setup(cache)
#     LeftFinger_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.LeftFingerTarget")
#     RightFinger_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.RightFingerTarget")
#     HandBase_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.HandBaseTarget")
#     RealRightFingerTarget_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.RealRightFingerTarget")
#     RealLeftFingerTarget_coord_id = get_compiled_coordID(cache, ".virtual_mechanism.RealLeftFingerTarget")

#     repulsiveField1_id = get_compiled_coordID(cache, ".virtual_mechanism.repulsiveField1")
#     repulsiveField2_id = get_compiled_coordID(cache, ".virtual_mechanism.repulsiveField2")
#     repulsiveField3_id = get_compiled_coordID(cache, ".virtual_mechanism.repulsiveField3")

#     RealHandBase_id = get_compiled_coordID(cache, ".robot.RealHandBase")

#     leftSpring_id = get_compiled_componentID(cache, "L spring")
#     rightSpring_id = get_compiled_componentID(cache, "R spring")
#     baseSpring_id = get_compiled_componentID(cache, "H spring")
#     RealRightSpring_id = get_compiled_componentID(cache, "RR spring")
#     RealLeftSpring_id = get_compiled_componentID(cache, "RL spring")
#     RealBaseSpring_id = get_compiled_componentID(cache, "RH spring")

#     positionDiff_id = get_compiled_coordID(cache, "RR pos error")

#     damper_R_id = get_compiled_componentID(cache, "R damper")
#     damper_RR_id = get_compiled_componentID(cache, "RR damper")
#     damper_L_id = get_compiled_componentID(cache, "L damper")
#     damper_RL_id = get_compiled_componentID(cache, "RL damper")
#     damper_H_id = get_compiled_componentID(cache, "H damper")
#     damper_RH_id = get_compiled_componentID(cache, "RH damper")

#     LeftFingerRobotFurther_coord_id = get_compiled_coordID(cache, ".robot.LeftFinger")
#     RightFingerRobotFurther_coord_id = get_compiled_coordID(cache, ".robot.RightFinger")
#     BaseRobotFurther_coord_id = get_compiled_coordID(cache, ".robot.HandBase")

#     RealRightFingerRobot_coord_id = get_compiled_coordID(cache, ".robot.RealLeftFinger")
#     RealLeftFingerRobot_coord_id = get_compiled_coordID(cache, ".robot.RealRightFinger")
#     RealBaseRobot_coord_id = get_compiled_coordID(cache, ".robot.RealHandBase")

#     leftDamper_coordDiff_id = get_compiled_coordID(cache, "L pos error")
#     rightDamper_coordDiff_id = get_compiled_coordID(cache, "R pos error")
#     baseDamper_coordDiff_id = get_compiled_coordID(cache, "H pos error")

#     additionalLeftSpring_id = get_compiled_componentID(cache, "AL spring")
#     additionalRightSpring_id = get_compiled_componentID(cache, "AR spring")
#     additionalBaseSpring_id = get_compiled_componentID(cache, "AH spring")

#     # return (LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id, RealRightFingerTarget_coord_id,
#     #          repulsiveField1_id, repulsiveField2_id, repulsiveField3_id, RealRightSpring_id,
#     #          leftSpring_id, rightSpring_id, baseSpring_id, RealHandBase_id)
#     return (LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id, RealRightFingerTarget_coord_id,
#         RealLeftFingerTarget_coord_id, repulsiveField1_id, repulsiveField2_id, repulsiveField3_id,
#         RealRightSpring_id, RealLeftSpring_id, leftSpring_id, rightSpring_id, baseSpring_id, RealHandBase_id, RealBaseSpring_id,
#         positionDiff_id, damper_R_id, damper_RR_id, damper_L_id, damper_RL_id, damper_H_id, damper_RH_id)
# end

# function f_control(cache, target_positions, t, setup_ret, extra)
#     LeftFinger_coord_id, RightFinger_coord_id, HandBase_coord_id, RealRightFingerTarget_coord_id,
#     RealLeftFingerTarget_coord_id, repulsiveField1_id, repulsiveField2_id, repulsiveField3_id,
#     RealRightSpring_id, RealLeftSpring_id, leftSpring_id, rightSpring_id, baseSpring_id, RealHandBase_id, RealBaseSpring_id,
#     positionDiff_id, damper_R_id, damper_RR_id, damper_L_id, damper_RL_id, damper_H_id, damper_RH_id = setup_ret

#     cache[LeftFinger_coord_id].coord_data.val[] = SVector(target_positions[1], target_positions[2], target_positions[3])
#     cache[RightFinger_coord_id].coord_data.val[] = SVector(target_positions[4], target_positions[5], target_positions[6])
#     cache[HandBase_coord_id].coord_data.val[] = SVector(target_positions[7], target_positions[8], target_positions[9])
#     cache[RealRightFingerTarget_coord_id].coord_data.val[] = SVector(target_positions[10], target_positions[11], target_positions[12])
#     cache[RealLeftFingerTarget_coord_id].coord_data.val[] = SVector(target_positions[13], target_positions[14], target_positions[15])

#     cache[repulsiveField1_id].coord_data.val[] = SVector(target_positions[16], target_positions[17], target_positions[18])
#     cache[repulsiveField2_id].coord_data.val[] = SVector(target_positions[19], target_positions[20], target_positions[21])
#     cache[repulsiveField3_id].coord_data.val[] = SVector(target_positions[22], target_positions[23], target_positions[24])

#     baseGoal = SVector(target_positions[25], target_positions[26], target_positions[27])

#     # cache[repulsiveField1_id].coord_data.val[] = SVector(target_positions[10], target_positions[11], target_positions[12])
#     # cache[repulsiveField2_id].coord_data.val[] = SVector(target_positions[13], target_positions[14], target_positions[15])
#     # cache[repulsiveField3_id].coord_data.val[] = SVector(target_positions[16], target_positions[17], target_positions[18])
#     # baseGoal = SVector(target_positions[19], target_positions[20], target_positions[21])

#     damping_val = norm(configuration(cache, positionDiff_id))
#     # print(damping_val)
#     # print("\n")
#     new_damping = 2*tanh(100*damping_val) + tanh(10*(damping_val-0.3)) + tanh(10*(damping_val+0.3))
#     # new_damping = 7*tanh(2*damping_val)

#     if norm(baseGoal) > 0.8
#         cache[leftSpring_id] = remake(cache[leftSpring_id]; stiffness=0.01)
#         cache[rightSpring_id] = remake(cache[rightSpring_id]; stiffness=0.01)
#         cache[baseSpring_id] = remake(cache[baseSpring_id]; stiffness=0.01)
#         cache[RealRightSpring_id] = remake(cache[RealRightSpring_id]; stiffness=0.01)
#         cache[RealLeftSpring_id] = remake(cache[RealLeftSpring_id]; stiffness=0.01)
#         cache[RealBaseSpring_id] = remake(cache[RealBaseSpring_id]; stiffness=0.01)
#     else
#         cache[leftSpring_id] = remake(cache[leftSpring_id]; stiffness=default_stiffness)
#         cache[rightSpring_id] = remake(cache[rightSpring_id]; stiffness=default_stiffness)
#         cache[baseSpring_id] = remake(cache[baseSpring_id]; stiffness=default_stiffness)
#         cache[RealRightSpring_id] = remake(cache[RealRightSpring_id]; stiffness=0.001)
#         cache[RealLeftSpring_id] = remake(cache[RealLeftSpring_id]; stiffness=0.001)
#         cache[RealBaseSpring_id] = remake(cache[RealBaseSpring_id]; stiffness=0.001)

#         cache[damper_R_id] = remake(cache[damper_R_id]; damping=new_damping)
#         cache[damper_RR_id] = remake(cache[damper_RR_id]; damping=new_damping)
#         cache[damper_L_id] = remake(cache[damper_L_id]; damping=new_damping)
#         cache[damper_RL_id] = remake(cache[damper_RL_id]; damping=new_damping)
#         cache[damper_H_id] = remake(cache[damper_H_id]; damping=new_damping)
#         cache[damper_RH_id] = remake(cache[damper_RH_id]; damping=new_damping)

#         # print(norm(configuration(cache, positionDiff_id)))
#         # print("\n")
#         # cache[RealRightSpring_id] = remake(cache[RealRightSpring_id]; stiffness=default_stiffness)
#     end
#     nothing 
# end

cvms = compile(vms)

qᵛ = Float64[]
with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 7, 14, 10) do connection
    ros_vm_controller(connection, cvms, qᵛ; f_control, f_setup, E_max=30.0)
end

# tspan = (0., 20π)
# dcache = new_dynamics_cache(compile(vms))
# q = ([0.0, 0.3, 0.0, -1.8, 0.0, π/2, 0.0], Float64[])
# q̇ = zero_q̇(dcache.vms)
# g = VMRobotControl.DEFAULT_GRAVITY
# prob = get_ode_problem(dcache, g, q, q̇, tspan)
# @info "Simulating robot handover sample."
# sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-3, reltol=1e-3); # Low tol to speed up simulation

# # ## Plotting
# # We create a figure with two scenes, for two different camera angles. We plot the robot, the
# # targets, the TCPs, and the obstacles in the scene.
# # We use observables for the time and the kinematics cache, which will be updated in the function
# # `animate_robot_odesolution`, causing any plots that depend upon these observables to be updated.
# fig = Figure(size = (720, 720), figure_padding=0)
# display(fig)
# ls = LScene(fig[1, 1]; show_axis=false)
# cam = cam3d!(ls, camera=:perspective, center=false)
# cam.lookat[] = [0., 0., 0.3]
# cam.eyeposition[] = [1.5, 0., 0.3]
# plotting_t = Observable(0.0)
# plotting_kcache = Observable(new_kinematics_cache(compile(vms)))

# target_scatter_kwargs = (;
#     color=:green, 
#     marker=:+, 
#     markersize=15, 
#     label="Targets",
#     transparency=true ## Avoid ugly white outline artefact on markers
# )
# tcp_scatter_kwargs = (;
#     color=:blue, 
#     marker=:x, 
#     markersize=15, 
#     label="TCPs",
#     transparency=true ## Avoid ugly white outline artefact on markers
# )


# ## Show robot
# robotvisualize!(ls, plotting_kcache;)
# #robotsketch!(ls, plotting_kcache, scale=0.3, linewidth=2.5, transparency=true)

# ## Label target and TCP
# target_1_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.LeftFingerTarget")
# target_2_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.RightFingerTarget")
# target_3_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.HandBaseTarget")

# l_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.LeftFinger")
# r_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.RightFinger")
# h_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.HandBase")

# scatter!(ls, plotting_kcache, [target_1_pos_id, target_2_pos_id, target_3_pos_id]; target_scatter_kwargs...)
# scatter!(ls, plotting_kcache, [l_tcp_pos_id, r_tcp_pos_id, h_tcp_pos_id]; tcp_scatter_kwargs...)

# savepath = joinpath(module_path, "docs/src/assets/random_trial.mp4")
# animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t, fastforward=1.0, fps=20)
