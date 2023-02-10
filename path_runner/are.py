planner_id: LIN
group_name: name of the planning group
max_velocity_scaling_factor: scaling factor of maximal Cartesian translational/rotational velocity
max_acceleration_scaling_factor: scaling factor of maximal Cartesian translational/rotational acceleration/deceleration start_state/joint_state/(name, position and velocity: joint name/position of the start state.
goal_constraints (goal can be given in joint space or Cartesian space)
    for a goal in joint space
        goal_constraints/joint_constraints/joint_name: goal joint name
        goal_constraints/joint_constraints/position: goal joint position
    for a goal in Cartesian space
        goal_constraints/position_constraints/header/frame_id: frame this data is associated with
        goal_constraints/position_constraints/link_name: target link name
        goal_constraints/position_constraints/constraint_region: bounding volume of the target point
        goal_constraints/position_constraints/target_point_offset: offset (in the link frame) for the target point on the target link (optional)