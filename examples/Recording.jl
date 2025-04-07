using StaticArrays
using VMRobotControl
using JLD2

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

    damping_val = norm(configuration(cache, positionDiff_id))
    new_damping = 2*tanh(100*damping_val) + tanh(10*(damping_val-0.3)) + tanh(10*(damping_val+0.3))

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
        cache[additionalLeftSpring_id] = remake(cache[additionalLeftSpring_id]; stiffness=default_stiffness)
        cache[additionalRightSpring_id] = remake(cache[additionalRightSpring_id]; stiffness=default_stiffness)
        cache[additionalBaseSpring_id] = remake(cache[additionalBaseSpring_id]; stiffness=default_stiffness)

        cache[damper_R_id] = remake(cache[damper_R_id]; damping=new_damping)
        cache[damper_L_id] = remake(cache[damper_L_id]; damping=new_damping)
        cache[damper_H_id] = remake(cache[damper_H_id]; damping=new_damping)
    end

    nothing 
end

IF_SAVE_EXP_DATA = true

include("panda_handover_v1.jl")

# try
#     include("panda_handover_v1.jl")
# catch e
#     if IF_SAVE_EXP_DATA
#         @save "first_trial.jld2" time_vector LeftFingerTarget_intended RightFingerTarget_intended BaseTarget_intended LeftFingerTarget_actual RightFingerTarget_actual BaseTarget_actual LeftFinger_intended RightFinger_intended Base_intended LeftFinger_actual RightFinger_actual Base_actual RepulsivePosition_right RepulsivePosition_left RepulsivePosition_front RightDamper_velocity LeftDamper_velocity BaseDamper_velocity HandPositions
#         println("exp data saved")
#         # EE_positions, EE_quats = transforms_to_quaternion(EE_transforms)
#         # plot_exp_data_position(result_foldername,times,knife_tip_positions,knife_back_positions,knife_tip_target_positions,knife_back_target_positions, EE_positions, xlim=[0,10])
#     end
#     println("Rocking experiment finished with: ", e)
# end