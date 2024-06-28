import sys
import rospy
import tf
from math import sqrt, pi, dist, fabs, cos
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import moveit_commander
import moveit_msgs.msg
import geometry_msgs
from geometry_msgs.msg import Pose

from colorama import Fore, Style

def all_close(goal, actual, tolerance): #Copied from moveit official tutorial
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = moveit_commander.conversions.pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = moveit_commander.conversions.pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class Robot(object):
    def __init__(self) -> None:
        super(Robot, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        # print(f"============ Printing robot state\n{self.robot.get_current_state()}")
        pass

    # def oscillate(self):
    #     time_period = 20
    #     i = 0
    #     rate = rospy.Rate(1)
    #     for i in range(20):
    #         # We get the joint values from the group and change some of the values:
    #         joint_goal = self.move_group.get_current_joint_values()
    #         joint_goal[0] = 0
    #         joint_goal[1] = 0
    #         joint_goal[2] = 0
    #         # joint_goal[3] = pi/2 * cos(tau * i / time_period)
    #         joint_goal[3] = ((-1) ** i) * pi / 2
    #         print(f"{Fore.GREEN}{i}{Style.RESET_ALL}")
    #         i += 1

    #         # The go command can be called with joint values, poses, or without any
    #         # parameters if you have already set the pose or joint target for the group
    #         self.move_group.go(joint_goal, wait=True)
    #         # rate.sleep()

    #     # Calling ``stop()`` ensures that there is no residual movement
    #     self.move_group.stop()
    #     print(f"{Fore.CYAN}Done{Style.RESET_ALL}")

    def go_to_pose_goal(self):
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def _go_axis_x(self, val: int):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x += val
        # pose_goal.position.y += 1e-6
        # pose_goal.position.z += 1e-6
        # pose_goal.orientation.x = 1e-6
        # pose_goal.orientation.y = 1e-6
        # pose_goal.orientation.z = 1e-6
        # pose_goal.orientation.w = 1

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        
        # For checking how much it differs
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_positive_x(self):
        return self._go_axis_x(0.01)
    
    def go_negative_x(self):
        return self._go_axis_x(-0.01)

def main():
    bot = Robot()
    inp = '1'
    while not rospy.is_shutdown():
        inp = input(f"{Fore.CYAN}Enter\n- '1' to oscilate\n- '2' to go_positive_x\n- '3' to go_negative_x\nValue:")
        if int(inp) == 1:
            bot.oscillate()
        elif int(inp) == 2:
            print(f"{Fore.GREEN}{bot.go_positive_x()}{Style.RESET_ALL}")
        elif int(inp) == 3:
            print(f"{Fore.GREEN}{bot.go_negative_x()}{Style.RESET_ALL}")
        else:
            continue
        print((bot.move_group.get_current_pose().pose))
        print(moveit_commander.conversions.pose_to_list(bot.move_group.get_current_pose().pose))
        # rospy.Rate(1.0 / 1).sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass