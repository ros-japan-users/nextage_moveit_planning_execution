#!/usr/bin/env python

import moveit_commander
import rospy

if __name__ == '__main__':
    rospy.init_node("planning_execution")

    robot = moveit_commander.RobotCommander()
    
    print "=" * 10 + " Robot Groups:"
    print robot.get_group_names()

    print "=" * 10 + " Printing robot state"
    print robot.get_current_state()
    print "=" * 10 

    rarm = moveit_commander.MoveGroupCommander("right_arm")
    larm = moveit_commander.MoveGroupCommander("left_arm")
    
    print "=" * 10 + " Right " + "=" * 10
    print "=" * 10 + " Reference frame: %s" % rarm.get_planning_frame()
    
    print "=" * 10 + " Reference frame: %s" % rarm.get_end_effector_link()
    
    rarm_initial_pose = rarm.get_current_pose().pose
    print "=" * 10 + " Printing initial pose: "
    print rarm_initial_pose
    
    print "=" * 10 + " Moving to a pose goal"
    rarm.set_pose_target([0.2035, -0.5399, 0.0709, 0, -1.6, 0])
    rarm.go()
    rospy.sleep(1)

    print "=" * 10 + " Left " + "=" * 10
    print "=" * 10 + " Reference frame: %s" % larm.get_planning_frame()
    print "=" * 10 + " Reference frame: %s" % larm.get_end_effector_link()

    larm_initial_pose = larm.get_current_pose().pose
    print "=" * 10 + " Printing initial pose: "
    print larm_initial_pose
    larm.set_pose_target([0.2035, 0.5399, 0.0709, 0, -1.6, 0])
    larm.go()
    rospy.sleep(1)
    
    print "Moving to an initial pose"
    rarm.set_pose_target(rarm_initial_pose)
    larm.set_pose_target(larm_initial_pose)
    rarm.go()
    larm.go()
    
    rospy.sleep(2)

