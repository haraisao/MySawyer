#
#
from __future__ import print_function
import rospy
import intera_interface
import numpy as np
import threading

#
#
from intera_core_msgs.msg import InteractionControlCommand
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)

from intera_motion_msgs.msg import TrajectoryOptions
from std_msgs.msg import String

#from intera_motion_interface.utility_functions import int2bool

#
#
#
class MySawyer(object):
  #
  #  Init class
  def __init__(self, name='MySawyer', limb='right'):
    rospy.init_node(name)
    rospy.sleep(1)
    #
    #
    self._limb=intera_interface.Limb(limb)
    self._head=intera_interface.Head()

    self._light=SawyerLight()
    
    self._display=intera_interface.HeadDisplay()
    self._cuff=intera_interface.Cuff()

    try:
      self._gripper=intera_interface.get_current_gripper_interface()
      self._is_clicksmart = isinstance(self._gripper, intera_interface.SimpleClickSmartGripper)
      if self._is_clicksmart:
        if self._gripper.needs_init():
          self._gripper.initialize()
      else:
        if not (self._gripper.is_calibrated() or
                self._gripper.calibrate() == True):
          raise

    except:
      self._gripper=None
      self._is_clicksmart=False
      
    #
    # Default Variables
    self._init_pos=[0.0, -1.178, 0.0, 2.178, 0.0, 0.567, 3.313]
    self._default_pos=[0.0, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0]
    self._speed_ratio=0.1 # 0.001 -- 1.0
    self._max_speed_ratio=0.5 # 0.001 -- 1.0
    self._max_accel_ratio=0.5 # 0.001 -- 1.0
    self._trajType='JOINT' # 'JOINT' ot 'CARTESIAN'
    self._interaction_active=True 
    self._K_impedance=[1300.0,1300.0, 1300.0, 30.0, 30.0, 30.0]
    self._max_impedance=[1,1,1,1,1,1]
    self._interaction_control_mode=[1,1,1,1,1,1]
    self._interaction_frame=[0,0,0,1,0,0,0]
    self._in_endpoint_frame=False
    self._endpoint_name='right_hand'
    self._force_command=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self._K_nullspace=[5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0]
    self._disable_damping_in_force_control=False
    self._disable_reference_resetting=False
    self._rotations_for_constrained_zeroG=False
    self._timeout=None

    # for Cartesian Pose
    self._in_tip_frame=False
    self._tip_name='right_hand'
    self._linear_speed=0.6     # m/s
    self._linear_accel=0.6     # m/s/s
    self._rotational_speed=1.57  # rad/s
    self._rotational_accel=1.57  # rad/s/s
    #
    #  Enable Robot
    self._rs=intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    self._init_state=self._rs.state().enabled
    self._rs.enable()

    ## for event handlers
    self._navigator=intera_interface.Navigator()
    self.ok_id=None
    self.show_id=None
    self.back_id=None

    #
    #
    self._angles=self._limb.joint_angles()
    self._pose=self._limb.endpoint_pose()

    self._joint_names=self._limb.joint_names()
    self._motion_trajectory=None
    #
    #

    self._limb.set_joint_position_speed(self._speed_ratio)

    self._motions={}
    self._joint_positions={'init':self._init_pos, 'default':self._default_pos}
    self._index=0
    self._p_index=0
    self._is_recording=False
    self.max_record_time=30
    self._accuracy=0.05
    self._recording_intval=0.5

    # velicity control mode
    self._running=True
    self._target=[-1, -1.4, 0, 2, 0, 0.5, 3.3]
    self._vctrl_intval=0.1
    self._vmax=0.3
    self._vrate=1.0

    #
    # LED white ON
    self._light.head_on()
    self.mkRosPorts()

  #
  #
  def mkRosPorts(self):
    self._sub=dict()
    self._pub=dict()

    self.sub['target_joint_pos']=rospy.Subscriber('target_joint_pos', String,self.set_target_joint_pos)

    self.pub['current_joint_pos']=rospy.Publisher('current_joint_pos', String,queu_size=1)



  #
  #
  def enable(self):
    self._rs.enable()
  #
  #
  def state(self):
    print(self._rs.state())
  #
  #
  def reset(self):
    self._rs.reset()
  #
  #
  def disable(self):
    self._rs.disable()
  #
  #
  def stop(self):
    self._rs.stop()

  #
  #
  def update_pose(self):
    self._angles=self._limb.joint_angles()
    self._pose=self._limb.endpoint_pose()
 
  #
  #
  def init_pos(self, use_motion_ctrl=True):
    if use_motion_ctrl:
      self.move_to([self._init_pos])
    else:
      self._light.head_green()
      self._limb.move_to_neutral(speed=self._speed_ratio)
      self.update_pose()
      self._light.head_on()
  #
  #
  def set_speed(self, rate=0.3):
    self._speed_ratio=rate
    self._limb.set_joint_position_speed(rate)

  #
  #
  def print_joiint_pos(self, dtime=5.0, intval=0.1):
    end_time = rospy.Time.now() + rospy.Duration(dtime) 
    while rospy.Time.now() < end_time:
      if rospy.is_shutdown() : break
      print(self._limb.endpoint_pose())
      rospy.sleep(intval)

  ##############################################
  # Joint Position Control (Depreciated for Intera 5.2 and beyond)
  def move_joints(self, pos):
    self._light.head_green()
    self._limb.move_to_joint_positions(pos)
    self.update_pose()
    self._light.head_on()
  #
  #
  def move_cart(self, x_dist, y_dist, z_dist):
    self._pose=self._limb.endpoint_pose()
    self._pose.position.x += x_dist
    self._pose.position.y += y_dist
    self._pose.position.z += z_dist
    self.move_joints(self._limb.ik_request(self._pose))
  #
  #
  def record_motion(self, name=None, dtime=0, intval=1.0):
    if not name :
      name=self.mk_motion_name()
      self._index += 1

    if dtime <= 0:
      dtime=self.max_record_time

    print ("Start Recording:", name)
    self._light.head_blue()

    self._motions[name]=[]
    self._is_recording=True
    end_time = rospy.Time.now() + rospy.Duration(dtime) 

    while (rospy.Time.now() < end_time) and self._is_recording :
      if rospy.is_shutdown() : break
      self._motions[name].append(self._limb.joint_angles())
      rospy.sleep(intval)

    print ("End Recording: record ", len(self._motions[name]), " points")
    self._is_recording=False
    self._light.head_on()
  #
  #
  def mk_motion_name(self):
    name = 'Motion_' + str(self._index)
    while name in self._motions:
      self._index += 1 
      name = 'Motion_' + str(self._index)
    return name

  def record_pos(self,val):
    if val :
      self._light.head_yellow()
      name = 'P_' + str(self._p_index)
      while name in self._joint_positions:
        self._p_index += 1 
        name = 'P_' + str(self._p_index)
      self._joint_positions[name]=self._limb.joint_ordered_angles()
      self._light.head_on()

  #
  #  Motion Recorder Event Handleer
  def start_record(self, value):
     if value:
         print('Start..')
         self.record_motion(None, 0, self._recording_intval)
  #
  #
  def stop_record(self, value):
     if value:
         print('Stop..')
         self._is_recording=False
  #
  #  set Handler
  def set_record(self):
     print ("Register callbacks")
     self.ok_id=self._navigator.register_callback(self.start_record, 'right_button_ok')
     self.back_id=self._navigator.register_callback(self.stop_record, 'right_button_back')
     self.square_id=self._navigator.register_callback(self.record_pos, 'right_button_square')
     self.show_id=self._navigator.register_callback(self.unset_record, 'right_button_show')
  #
  # unset Handler
  def unset_record(self, value=0):
    if value and self.ok_id :
      print ("Unregister all callbacks")
      if self._navigator.deregister_callback(self.ok_id) : self.ok_id=None
      #if self._navigator.deregister_callback(self.show_id) : self.show_id=None
      if self._navigator.deregister_callback(self.back_id) : self.back_id=None
      if self._navigator.deregister_callback(self.square_id) : self.square_id=None
  
  #######################################################
  #
  #
  def play_motion(self, name, intval=0.0):
    self._limb.set_joint_position_speed(self._speed_ratio)
    self._light.head_green()
    for pos in self._motions[name]:
      if rospy.is_shutdown() :
        self._light.head_red()
        return
      # 
      self._limb.move_to_joint_positions(pos, threshold=self._accuracy)
      if intval > 0: rospy.sleep(intval)
    self._light.head_on()
  #
  #
  def play_motion_seq(self, names):
    self._limb.set_joint_position_speed(self._speed_ratio)
    self._light.head_green()
    for name in names:
      for pos in self._motions[name]:
        if rospy.is_shutdown() :
          self._light.head_red()
          return
        self._limb.move_to_joint_positions(pos)
    self._light.head_on()
  #
  #
  def list_motions(self):
    print(self._motions.keys())
  #
  #
  def joint_pos_d2l(self, pos):
    return map(lambda x: pos[x], self._joint_names)

  def convert_motion(self, name):
    return map(lambda x: self.joint_pos_d2l(x), self._motions[name])

  #############################################
  #
  #
  def save_motion(self, name):
    with open("motions/"+name+".jpos", mode="w") as f:
      for pos in self._motions[name]:
        f.write(str(pos))
        f.write("\n")
  #
  #
  def load_motion(self, name):
    self._motions[name]=[]
    with open("motions/"+name+".jpos") as f:
      motion=f.readlines()
    for p in motion:
      self._motions[name].append( eval(p) )


  def get_joint_positions(self, name):
    if type(name) == str:
      if name in self._joint_positions:
        target_joints=self._joint_positions[name]
      else:
        print("Invalid position name")
        target_joints=None
    elif len(name) == 7:
      target_joints=name
    return  target_joints

  ####################################
  #
  #  Move Motion
  def move_to(self, name=None, tout=None, with_in_contact=False, wait_for_result=True):
    #
    # for Motion Controller Interface
    if type(name) == str and name in self._motions:
      waypoints=self.convert_motion(name)
    elif type(name) == list:
      waypoints=name
    else:
      print("Invalid motion name")
      return None

    self._motion_trajectory=MotionTrajectory(limb=self._limb)

    _wpt_opts=MotionWaypointOptions(max_joint_speed_ratio=self._max_speed_ratio,
                                       max_joint_accel=self._max_accel_ratio)
    _waypoint=MotionWaypoint(options=_wpt_opts, limb=self._limb)

    #
    # set current joint position...
    _waypoint.set_joint_angles(joint_angles=self._limb.joint_ordered_angles())
    self._motion_trajectory.append_waypoint(_waypoint.to_msg())

    #
    # set target joint position...
    for pos in waypoints:
      if type(pos) == str:
        if pos in self._joint_positions:
          pos=self._joint_positions[pos]
          _waypoint.set_joint_angles(joint_angles=pos)
          self._motion_trajectory.append_waypoint(_waypoint.to_msg())
      else:
        _waypoint.set_joint_angles(joint_angles=pos)
        self._motion_trajectory.append_waypoint(_waypoint.to_msg())

    #
    #
    if with_in_contact :
      opts=self.get_in_contact_opts()
      if opts :
        self._motion_trajectory.set_trajectory_options(opts)

    #
    # run motion...
    self._light.head_green()
    result=self._motion_trajectory.send_trajectory(wait_for_result=wait_for_result,timeout=tout)

    #
    #
    if result is None:
      self._light.head_yellow()
      print("Trajectory FAILED to send")
      return None
    #
    #
    if not wait_for_result : return True

    #
    if result.result: self._light.head_on()
    else: self._light.head_red()

    #
    #
    self._motion_trajectory=None
    return result.result

  #
  #
  def cart_move_to(self, target_pos, tout=None, relative_mode=False,  wait_for_result=True):
    #
    # for Motion Controller Interface
    _trajectory_opts=TrajectoryOptions()
    _trajectory_opts.interpolation_type=TrajectoryOptions.CARTESIAN

    #
    self._motion_trajectory=MotionTrajectory(trajectory_options=_trajectory_opts, limb=self._limb)

    #
    _wpt_opts=MotionWaypointOptions(max_linear_speed=self._linear_speed,
                                       max_linear_accel=self._linear_accel,
                                       max_rotational_speed=self._rotational_speed,
                                       max_rotational_accel=self._rotational_accel,
                                       max_joint_speed_ratio=1.0)
    _waypoint=MotionWaypoint(options=_wpt_opts, limb=self._limb)

    #
    endpoint_state=self._limb.tip_state(self._tip_name)
    pose=endpoint_state.pose

    ########################################
    #  set target position

    if relative_mode:
      # relative position : target_pos -> x, y, z, roll, pitch, yew
      trans = PyKDL.Vector(target_pos[0],target_pos[1],target_pos[2])
      rot = PyKDL.Rotation.RPY(target_pos[3], target_pos[4],target_pos[5])
      f2 = PyKDL.Frame(rot, trans)

      if self._in_tip_frame:
        # endpoint's cartesian systems
        pose=posemath.toMsg(posemath.fromMsg(pose) * f2)
      else:
        # base's cartesian systems
        pose=posemath.toMsg(f2 * posemath.fromMsg(pose))
    else:
      #  global position : x, y, z, rx, ry, rz, rw
      pose.position.x=target_pos[0]
      pose.position.y=target_pos[1]
      pose.position.z=target_pos[2]
      pose.orientation.x=target_pos[3]
      pose.orientation.y=target_pos[4]
      pose.orientation.z=target_pos[5]
      pose.orientation.w=target_pos[6]
    #
    ###########################################
    # set target position.
    poseStamped=PoseStamped()
    poseStamped.pose=pose
    _waypoint.set_cartesian_pose(poseStamped, self._tip_name, [])
    self._motion_trajectory.append_waypoint(_waypoint.to_msg())
    #
    # run motion...
    self._light.head_green()
    result=self._motion_trajectory.send_trajectory( wait_for_result=wait_for_result,timeout=tout)

    #
    if result is None:
      self._light.head_yellow()
      print("Trajectory FAILED to send")
      return None
    #
    if not wait_for_result : return True
    #
    if result.result: self._light.head_on()
    else: self._light.head_red()
    #
    self._motion_trajectory=None
    return result.result

  #
  # stop motion...
  def stop_trajectory(self):
    if self._motion_trajectory :
      self._motion_trajectory.stop_trajectory()

  #
  #  set Interaction control
  def set_interaction_params(self):
    interaction_options = InteractionOptions()
    interaction_options.set_interaction_control_active(self._interaction_active)
    interaction_options.set_K_impedance(self._K_impedance)
    interaction_options.set_max_impedance(self._max_impedance)
    interaction_options.set_interaction_control_mode(self._interaction_control_mode)
    interaction_options.set_in_endpoint_frame(self._in_endpoint_frame)
    interaction_options.set_force_command(self._force_command)
    interaction_options.set_K_nullspace(self._K_nullspace)
    interaction_options.set_endpoint_name(self._endpoint_name)

    if len(self._interaction_frame) == 7:
      quat_sum_square = self._interaction_frame[3]*self._interaction_frame[3] + self._interaction_frame[4]*self._interaction_frame[4] + self._interaction_frame[5]*self._interaction_frame[5] + self._interaction_frame[6]*self._interaction_frame[6]

      if quat_sum_square  < 1.0 + 1e-7 and quat_sum_square > 1.0 - 1e-7:
        interaction_frame = Pose()
        interaction_frame.position.x = self._interaction_frame[0]
        interaction_frame.position.y = self._interaction_frame[1]
        interaction_frame.position.z = self._interaction_frame[2]
        interaction_frame.orientation.w = self._interaction_frame[3]
        interaction_frame.orientation.x = self._interaction_frame[4]
        interaction_frame.orientation.y = self._interaction_frame[5]
        interaction_frame.orientation.z = self._interaction_frame[6]
        interaction_options.set_interaction_frame(interaction_frame)
      else:
        print('Invalid input to quaternion! The quaternion must be a unit quaternion!')
        return None

    else:
        print('Invalid input to interaction_frame!')
        return None

    interaction_options.set_disable_damping_in_force_control(self._disable_damping_in_force_control)
    interaction_options.set_disable_reference_resetting(self._disable_reference_resetting)
    interaction_options.set_rotations_for_constrained_zeroG(self._rotations_for_constrained_zeroG)
    return interaction_options

  #
  #
  def set_interaction_mode(self):
    pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size = 1)
    interaction_options = self.set_interaction_params()
    if interaction_options:
      msg=interaction_options.to_msg()
      pub.publish(msg)

  #
  #
  def get_in_contact_opts(self):
    interaction_options = self.set_interaction_params()
    if interaction_options:
      trajectory_options = TrajectoryOptions()
      trajectory_options.interaction_control = True
      trajectory_options.interpolation_type = self._trajType
      trajectory_options.interaction_params = interaction_options.to_msg()
      return trajectory_options
    else:
      return None
  #
  #
  def gripper_open(self):
    if self._gripper and self._gripper.is_ready():
      if self._is_clicksmart:
        self._gripper.set_ee_signal_value('grip', False)
      else:
        self._gripper.open()
  #
  #
  def gripper_close(self):
    if self._gripper and self._gripper.is_ready():
      if self._is_clicksmart:
        self._gripper.set_ee_signal_value('grip', True)
      else:
        self._gripper.close()
  #
  #
  def terminate(self):
    self._running=False
    self._vctrl_th.join()
    self._vctrl_th=None
  #
  # vctrl_loop
  def start_vctrl(self, hz=100):
    #self._vctrl_th=threading.Thread(target=vctrl_loop, args=(self,))
    self._vctrl_th=threading.Thread(target=self.vctrl_loop, args=(hz,self.report,))
    self._running=True
    self._vctrl_th.start()

  def _vctrl_one_cycle(self, func=None):
    cmd={}
    cur=self._limb.joint_ordered_angles()
    dv=np.array(self._target) - np.array(cur)

    if np.linalg.norm(dv) < self._accuracy:
      if func:
        func(self)
    else:
      vels = map(lambda x: maxmin(x*self._vrate, self._vmax, -self._vmax), dv)  
      for i,name in enumerate(self._joint_names):
        cmd[name]=vels[i] 
      self._limb.set_joint_velocities(cmd)

  def vctrl_loop(self, hz, func=None):
    rate=rospy.Rate(hz)
    while self._running and (not rospy.is_shutdown()) :
      self._vctrl_one_cycle(func)
      rate.sleep()

    self._limb.exit_control_mode()
    print("Terminated")
   
  def set_target_joint_pos(self, data):
    self._target=eval(data.data)

  def report(self,val):
    cur=self._limb.joint_ordered_angles()
    self.pub['current_joint_pos'].publish(str(cur)) 

#
#  LED Light
#
class SawyerLight(object):
  def __init__(self):
    self._light=intera_interface.Lights()
  #
  #  Right
  ###########################
  def head_yellow(self):
    self._light.set_light_state('head_blue_light',False)
    self._light.set_light_state('head_red_light',True)
    self._light.set_light_state('head_green_light',True)
  #
  #
  def head_blue(self):
    self._light.set_light_state('head_red_light',False)
    self._light.set_light_state('head_green_light',False)
    self._light.set_light_state('head_blue_light',True)
  #
  #
  def head_green(self):
    self._light.set_light_state('head_red_light',False)
    self._light.set_light_state('head_blue_light',False)
    self._light.set_light_state('head_green_light',True)
  #
  #
  def head_red(self):
    self._light.set_light_state('head_green_light',False)
    self._light.set_light_state('head_blue_light',False)
    self._light.set_light_state('head_red_light',True)
  #
  #
  def head_on(self):
    self._light.set_light_state('head_red_light',True)
    self._light.set_light_state('head_green_light',True)
    self._light.set_light_state('head_blue_light',True)
  #
  #
  def head_off(self):
    self._light.set_light_state('head_red_light',False)
    self._light.set_light_state('head_green_light',False)
    self._light.set_light_state('head_blue_light',False)

  ###########################


###
#
def maxmin(v, mx, mn):
  return np.max([np.min([v,mx]), mn])

def vctrl_loop(r,intval=0.1):
  cmd={}
  _pp=True
  while r._running and (not rospy.is_shutdown()) :
    cur=r._limb.joint_ordered_angles()
    dv=np.array(r._target) - np.array(cur)
    d=np.linalg.norm(dv)
    if d < 0.1:
      if _pp:
        print(cur)
        _pp=False
    else:
      vels = map(lambda x: maxmin(x*r._vrate, r._vmax, -r._vmax), dv)  
      for i,name in enumerate(r._joint_names):
        cmd[name]=vels[i] 
      r._limb.set_joint_velocities(cmd)
      _pp=True

    rospy.sleep(r._vctrl_intval)
  r._limb.exit_control_mode()
  print("Terminated")
   
