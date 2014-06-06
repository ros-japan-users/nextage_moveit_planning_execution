#!/usr/bin/env python

import moveit_commander
import rospy
import geometry_msgs.msg
import copy

if __name__ == '__main__':
    rospy.init_node("planning_execution")

    robot = moveit_commander.RobotCommander()
    
    print "=" * 10, " Robot Groups:"
    print robot.get_group_names()

    print "=" * 10, " Printing robot state"
    print robot.get_current_state()
    print "=" * 10 

    rarm = moveit_commander.MoveGroupCommander("right_arm")
    larm = moveit_commander.MoveGroupCommander("left_arm")

    print "=" * 15, " Right arm ", "=" * 15
    print "=" * 10, " Reference frame: %s" % rarm.get_planning_frame()
    
    print "=" * 10, " Reference frame: %s" % rarm.get_end_effector_link()
    
    rarm_initial_pose = rarm.get_current_pose().pose
    print "=" * 10, " Printing initial pose: "
    print rarm_initial_pose
    
    target_pose_r = geometry_msgs.msg.Pose()
    target_pose_r.position.x = 0.2035
    target_pose_r.position.y = -0.5399
    target_pose_r.position.z = 0.0709
    target_pose_r.orientation.x = 0.000427
    target_pose_r.orientation.y = 0.000317
    target_pose_r.orientation.z = -0.000384
    target_pose_r.orientation.w = 0.999999
    rarm.set_pose_target(target_pose_r)
    
    print "=" * 10, " plan1..."
    rarm.go()
    rospy.sleep(1)

    print "=" * 15, " Left arm ", "=" * 15
    print "=" * 10, " Reference frame: %s" % larm.get_planning_frame()
    print "=" * 10, " Reference frame: %s" % larm.get_end_effector_link()

    larm_initial_pose = larm.get_current_pose().pose
    print "=" * 10, " Printing initial pose: "
    print larm_initial_pose
    
    target_pose_l = [
        target_pose_r.position.x,
        -target_pose_r.position.y,
        target_pose_r.position.z,
        target_pose_r.orientation.x,
        target_pose_r.orientation.y,
        target_pose_r.orientation.z,
        target_pose_r.orientation.w
    ]

    larm.set_pose_target(target_pose_l)
    
    print "=" * 10, " plan2..."
    larm.go()
    rospy.sleep(1)
    
    print "=" * 10, " Planning to a joint-space goal"
    rarm.clear_pose_targets()
    print "=" * 10, " Joint values: ", rarm.get_current_joint_values()
    
    rarm_variable_values = [
        1.4377544509919726, 
        -1.3161643133168621, 
        -2.126307271452489, 
        1.4335761224859305, 
        0.02359653211486051, 
        0.55989121526186
    ]
    
    rarm.set_joint_value_target(rarm_variable_values)
    
    print "=" * 10, " plan3..."
    rarm.go()
    rospy.sleep(1)
    
    print "=" * 10, " Cartesian Paths"
    waypoints = []

    waypoints.append(larm.get_current_pose().pose)

    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x
    wpose.position.y = waypoints[0].position.y - 0.15
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.05
    waypoints.append(copy.deepcopy(wpose))
    
    (plan, fraction) = larm.compute_cartesian_path(waypoints, 0.01, 0.0)
    
    print "=" * 10, " plan4..."
    larm.execute(plan)
    rospy.sleep(5)
    
    print "=" * 10, " Moving to an initial pose"
    rarm.set_pose_target(rarm_initial_pose)
    larm.set_pose_target(larm_initial_pose)
    rarm.go()
    larm.go()
    rospy.sleep(2)

