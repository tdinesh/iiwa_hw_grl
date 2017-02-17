#!/usr/bin/env python
import rospy
from iiwa_msgs.srv import ConfigureSmartServo,ConfigureSmartServoRequest
from iiwa_msgs.msg import ControlMode, CartesianImpedanceControlMode, CartesianControlModeLimits, JointImpedanceControlMode

def main():
  rospy.init_node('iiwa_switch_control')

  #Setting Cartesian Impedance mode
  cartesian_impedance = CartesianImpedanceControlMode()
  limits = CartesianControlModeLimits()

  # Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad].
  # Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0.
  cartesian_impedance.cartesian_stiffness.x = 1000
  cartesian_impedance.cartesian_stiffness.y = 1000
  cartesian_impedance.cartesian_stiffness.z = 550
  cartesian_impedance.cartesian_stiffness.a = 300
  cartesian_impedance.cartesian_stiffness.b = 300
  cartesian_impedance.cartesian_stiffness.c = 300

  # Dimensionless damping values for the cartesian impedance control, for all degrees of freedom : [x, y, z, a, b, c].
  # Precondition: 0.1 <= x, y, z, a, b, c <= 1.0.
  cartesian_impedance.cartesian_damping.x = 0.7
  cartesian_impedance.cartesian_damping.y = 0.7
  cartesian_impedance.cartesian_damping.z = 0.7
  cartesian_impedance.cartesian_damping.a = 0.7
  cartesian_impedance.cartesian_damping.b = 0.7
  cartesian_impedance.cartesian_damping.c = 0.7

  # The stiffness value for null space [Nm/rad].
  # Precondition: 0.0 <= value.
  cartesian_impedance.nullspace_stiffness = 2.0;

  # The damping parameter for null space [Nm*s/rad].
  # Precondition: value >= 0.3 and value <= 1.0. - A good damping value is 0.7.
  cartesian_impedance.nullspace_damping = 0.7;

  # Sets the maximum cartesian deviation accepted. If the deviation is exceeded a stop condition occurs.
  # [x, y, z] in [mm]. Precondition: value > 0.0.
  # [a, b, c] in [rad]. Precondition: value > 0.0.
  limits.max_path_deviation.x = 1000
  limits.max_path_deviation.y = 1000
  limits.max_path_deviation.z = 100
  limits.max_path_deviation.a = 5.0
  limits.max_path_deviation.b = 5.0
  limits.max_path_deviation.c = 5.0

  # The maximum control force parameter.
  # [x, y, z] in [N]. Precondition: value > 0.0.
  # [a, b, c] in [Nm]. Precondition: value > 0.0.
  limits.max_control_force.x = 200
  limits.max_control_force.y = 200
  limits.max_control_force.z = 200
  limits.max_control_force.a = 200
  limits.max_control_force.b = 200
  limits.max_control_force.c = 200

  # Indicates whether a stop condition is fired if the maximum control force is exceeded.
  limits.max_control_force_stop = False

  # Maximum Cartesian velocity parameter
  # [x, y, z] in [mm/s]. Precondition: value > 0.0.
  # [a, b, c] in [rad/s]. Precondition: value > 0.0.
  limits.max_cartesian_velocity.x = 1000
  limits.max_cartesian_velocity.y = 1000
  limits.max_cartesian_velocity.z = 1000
  limits.max_cartesian_velocity.a = 6.3
  limits.max_cartesian_velocity.b = 6.3
  limits.max_cartesian_velocity.c = 6.3

  # Stiffness values in [Nm/rad]. Stiffness values must be >= 0.
  joint_impedance = JointImpedanceControlMode()
  joint_impedance.joint_stiffness.a1 = 400
  joint_impedance.joint_stiffness.a2 = 400
  joint_impedance.joint_stiffness.a3 = 400
  joint_impedance.joint_stiffness.a4 = 400
  joint_impedance.joint_stiffness.a5 = 400
  joint_impedance.joint_stiffness.a6 = 400
  joint_impedance.joint_stiffness.a7 = 400

  # Damping values. Damping values must be between 0 and 1.
  joint_impedance.joint_damping.a1 = 0.7
  joint_impedance.joint_damping.a2 = 0.7
  joint_impedance.joint_damping.a3 = 0.7
  joint_impedance.joint_damping.a4 = 0.7
  joint_impedance.joint_damping.a5 = 0.7
  joint_impedance.joint_damping.a6 = 0.7
  joint_impedance.joint_damping.a7 = 0.7

  control_mode = ControlMode.POSITION_CONTROL
  control_mode = ControlMode.JOINT_IMPEDANCE
  #control_mode = ControlMode.CARTESIAN_IMPEDANCE

  req = ConfigureSmartServoRequest()
  req.control_mode = control_mode
  req.cartesian_impedance = cartesian_impedance
  req.joint_impedance = joint_impedance
  req.limits = limits

  print 'calling controller switch'
  print req.control_mode

  try:
    switch_service = rospy.ServiceProxy('/iiwa/configuration/configureSmartServo',ConfigureSmartServo)
    sw_response = switch_service(req)
    print "response recieved: \n", sw_response
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

if __name__ == '__main__':
  main()


