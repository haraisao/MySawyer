#
#
from __future__ import print_function
import rospy
import intera_interface


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
    self._light=intera_interface.Lights()
    self._display=intera_interface.HeadDisplay()
    self._cuff=intera_interface.Cuff()
    try:
      self._gripper=intera_interface.Gripper()
    except:
      self._gripper=None
      

    #
    #  Enable Robot
    self._rs=intera_interface.RobotEnable()
    self._init_state=self._rs.state().enabled
    self._rs.enable()

    self._navigator=intera_interface.Navigator()
    self.ok_id=None
    self.show_id=None
    self.back_id=None
    #

    self._angles=self._limb.joint_angles()
    self._pose=self._limb.endpoint_pose()

    self._speed=0.15
    self._limb.set_joint_position_speed(self._speed)

    self._motions={}
    self._index=0
    self._is_recording=False
    self.max_record_time=30
    self._accuracy=0.05

    self.head_light_on()
  #
  #
  def update_pose(self):
    self._angles=self._limb.joint_angles()
    self._pose=self._limb.endpoint_pose()
 
  #
  #
  def init_pos(self):
    self.head_green()
    self._limb.move_to_neutral(speed=self._speed)
    self.update_pose()
    self.head_light_on()

  #
  #
  def move_joints(self, pos):
    self.head_green()
    self._limb.move_to_joint_positions(pos)
    self.update_pose()
    self.head_light_on()
   
  #
  #
  def set_speed(self, rate=0.3):
    self._speed=rate
    self._limb.set_joint_position_speed(rate)

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
  def print_pos(self, dtime=5.0, intval=0.1):
    end_time = rospy.Time.now() + rospy.Duration(dtime) 
    while rospy.Time.now() < end_time:
      if rospy.is_shutdown() : break
      print(self._limb.endpoint_pose())
      rospy.sleep(intval)

  #
  #
  def record_motion(self, name=None, dtime=0, intval=1.0):
    if not name :
      name=self.mk_motion_name()
      self._index += 1

    if dtime <= 0:
      dtime=self.max_record_time

    print ("Start Recording:", name)
    self.head_blue()

    self._motions[name]=[]
    self._is_recording=True
    end_time = rospy.Time.now() + rospy.Duration(dtime) 
    while (rospy.Time.now() < end_time) and self._is_recording :
      if rospy.is_shutdown() : break
      self._motions[name].append(self._limb.joint_angles())
      rospy.sleep(intval)

    print ("End Recording: record ", len(self._motions[name]), " points")
    self._is_recording=False
    self.head_light_on()

  def mk_motion_name(self):
    name = 'Motion_' + str(self._index)
    while name in self._motions:
      self._index += 1 
      name = 'Motion_' + str(self._index)
    return name
  #
  #######################################################
  def start_record(self, value):
     if value:
         print('Start..')
         self.record_motion(None, 0, 1.0)

  def stop_record(self, value):
     if value:
         print('Stop..')
         self._is_recording=False

  #
  #
  def set_record(self):
     print ("Register callbacks")
     self.ok_id=self._navigator.register_callback(self.start_record, 'right_button_ok')
     self.back_id=self._navigator.register_callback(self.stop_record, 'right_button_back')
     self.show_id=self._navigator.register_callback(self.unset_record, 'right_button_show')
     
  def unset_record(self, value=0):
     if value and self.ok_id :
       print ("Unregister all callbacks")
       if self._navigator.deregister_callback(self.ok_id) : self.ok_id=None
       if self._navigator.deregister_callback(self.show_id) : self.show_id=None
       if self._navigator.deregister_callback(self.back_id) : self.back_id=None
  #
  #
  def play_motion(self, name, intval=0.0):
    self.head_green()
    for pos in self._motions[name]:
      if rospy.is_shutdown() :
        self.head_red()
        return
      self._limb.move_to_joint_positions(pos, threshold=self._accuracy)
      if intval > 0: rospy.sleep(intval)
    self.head_light_on()
 
  #
  #
  def play_motion_seq(self, names):
    self.head_green()
    for name in names:
      for pos in self._motions[name]:
        if rospy.is_shutdown() :
          self.head_red()
          return
        self._limb.move_to_joint_positions(pos)
    self.head_light_on()
 
  #
  #
  def list_motions(self):
      print(self._motions.keys())
  
  #############################################
  #
  #
  def save_motion(self, name):
    with open(name+".pos", mode="w") as f:
      for pos in self._motions[name]:
        f.write(str(pos))
        f.write("\n")
  #
  #
  def load_motion(self, name):
    self._motions[name]=[]
    with open(name+".pos") as f:
      motion=f.readlines()
    for p in motion:
      self._motions[name].append( eval(p) )
  #
  #  Right
  ###########################
  def head_yellow(self):
    self._light.set_light_state('head_blue_light',False)
    self._light.set_light_state('head_red_light',True)
    self._light.set_light_state('head_green_light',True)

  def head_blue(self):
    self._light.set_light_state('head_red_light',False)
    self._light.set_light_state('head_green_light',False)
    self._light.set_light_state('head_blue_light',True)

  def head_green(self):
    self._light.set_light_state('head_red_light',False)
    self._light.set_light_state('head_blue_light',False)
    self._light.set_light_state('head_green_light',True)

  def head_red(self):
    self._light.set_light_state('head_green_light',False)
    self._light.set_light_state('head_blue_light',False)
    self._light.set_light_state('head_red_light',True)

  def head_light_on(self):
    self._light.set_light_state('head_red_light',True)
    self._light.set_light_state('head_green_light',True)
    self._light.set_light_state('head_blue_light',True)

  def head_light_off(self):
    self._light.set_light_state('head_red_light',False)
    self._light.set_light_state('head_green_light',False)
    self._light.set_light_state('head_blue_light',False)
