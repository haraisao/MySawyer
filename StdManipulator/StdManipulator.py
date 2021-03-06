#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file StdManipulator.py
 @brief Manipulator controller with JARA Standard Interfaces
 @date $Date$
 @license the MIT License

 copyright (C) 2018 Isao Hara,AIST,JP All rights reserved

"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import ManipulatorCommonInterface_Middle_idl
import ManipulatorCommonInterface_Common_idl

# Import Service implementation class
# <rtc-template block="service_impl">
from ManipulatorCommonInterface_Middle_idl_impl import *
from ManipulatorCommonInterface_Common_idl_impl import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>

from MySawyer import *

# This module's spesification
# <rtc-template block="module_spec">
stdmanipulator_spec = ["implementation_id", "StdManipulator", 
		 "type_name",         "StdManipulator", 
		 "description",       "Manipulator controller with JARA Standard Interfaces", 
		 "version",           "1.0.0", 
		 "vendor",            "AIST", 
		 "category",          "Manipulator", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
         	 "conf.default.vmax", "0.3",
		 "conf.default.vrate", "2.0",
		 "conf.default.accuracy", "0.01",
		 "conf.default.gripper_reverse", "1",

		 "conf.__widget__.vmax", "text",
		 "conf.__widget__.vrate", "text",
		 "conf.__widget__.accuracy", "text",
		 "conf.__widget__.gripper_reverse", "radio",
		 "conf.__constraints__.gripper_reverse", "(0,1)",

                 "conf.__type__.vmax", "float",
                 "conf.__type__.vrate", "float",
                 "conf.__type__.accuracy", "float",
                 "conf.__type__.gripper_reverse", "int",
		 ""]
# </rtc-template>

##
# @class StdManipulator
# @brief Manipulator controller with JARA Standard Interfaces
# 
# 
class StdManipulator(OpenRTM_aist.DataFlowComponentBase):
	
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		joints_arg = [None] * ((len(RTC._d_TimedFloatSeq) - 4) / 2)
		self._d_joints = RTC.TimedFloatSeq(*joints_arg)
		"""
		"""
		self._jointsIn = OpenRTM_aist.InPort("joints", self._d_joints)
		grip_arg = [None] * ((len(RTC._d_TimedOctet) - 4) / 2)
		self._d_grip = RTC.TimedOctet(*grip_arg)
		"""
		"""
		self._gripIn = OpenRTM_aist.InPort("grip", self._d_grip)
		out_joints_arg = [None] * ((len(RTC._d_TimedFloatSeq) - 4) / 2)
		self._d_out_joints = RTC.TimedFloatSeq(*out_joints_arg)
		"""
		"""
		self._out_jointsOut = OpenRTM_aist.OutPort("out_joints", self._d_out_joints)
		out_velocity_arg = [None] * ((len(RTC._d_TimedFloatSeq) - 4) / 2)
		self._d_out_velocity = RTC.TimedFloatSeq(*out_velocity_arg)
		"""
		"""
		self._out_velocityOut = OpenRTM_aist.OutPort("out_velocity", self._d_out_velocity)
		out_torque_arg = [None] * ((len(RTC._d_TimedFloatSeq) - 4) / 2)
		self._d_out_torque = RTC.TimedFloatSeq(*out_torque_arg)
		"""
		"""
		self._out_torqueOut = OpenRTM_aist.OutPort("out_torque", self._d_out_torque)

		"""
		"""
		self._mnipCommonPort = OpenRTM_aist.CorbaPort("mnipCommon")
		"""
		"""
		self._manipMiddlePort = OpenRTM_aist.CorbaPort("manipMiddle")

		"""
		"""
		self._JARA_ARM_ManipulatorCommonInterface_Common = ManipulatorCommonInterface_Common_i()
		"""
		"""
		self._JARA_ARM_ManipulatorCommonInterface_Middle = ManipulatorCommonInterface_Middle_i()
		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
                """

		 - Name:  vmax
		 - DefaultValue: 0.3
		"""
		self._vmax = [0.3]
		"""

		 - Name:  vrate
		 - DefaultValue: 2.0
		"""
		self._vrate = [2.0]
		"""

		 - Name:  accuracy
		 - DefaultValue: 0.01
		"""
		self._accuracy = [0.01]
               	"""

		 - Name:  gripper_reverse
		 - DefaultValue: True
		"""
		self._gripper_reverse = [1]

		
		# </rtc-template>


		 
	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
                self._robot=None
		# Bind variables and configuration variable
          	self.bindParameter("vmax", self._vmax, "0.3")
		self.bindParameter("vrate", self._vrate, "2.0")
		self.bindParameter("accuracy", self._accuracy, "0.01")
		self.bindParameter("gripper_reverse", self._gripper_reverse, "1")
		
		# Set InPort buffers
		self.addInPort("joints",self._jointsIn)
		self.addInPort("grip",self._gripIn)
		
		# Set OutPort buffers
		self.addOutPort("out_joints",self._out_jointsOut)
		self.addOutPort("out_velocity",self._out_velocityOut)
		self.addOutPort("out_torque",self._out_torqueOut)
		
		# Set service provider to Ports
		self._mnipCommonPort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._JARA_ARM_ManipulatorCommonInterface_Common)
		self._manipMiddlePort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._JARA_ARM_ManipulatorCommonInterface_Middle)
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		self.addPort(self._mnipCommonPort)
		self.addPort(self._manipMiddlePort)
		
		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
		##
		#
		# The activated action (Active state entry action)
		# former rtc_active_entry()
		#
		# @param ec_id target ExecutionContext Id
		# 
		# @return RTC::ReturnCode_t
		#
		#
	def onActivated(self, ec_id):
                self._robot=MySawyer(self.getInstanceName(), anonymous=False)
                self._robot.activate()
                self._robot._is_running=True
		self._JARA_ARM_ManipulatorCommonInterface_Common._robot=self._robot
		self._JARA_ARM_ManipulatorCommonInterface_Middle._robot=self._robot
	
		return RTC.RTC_OK
	
		##
		#
		# The deactivated action (Active state exit action)
		# former rtc_active_exit()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onDeactivated(self, ec_id):
                self._robot._is_running=False
                self._robot.disable()
	
		return RTC.RTC_OK
	
		##
		#
		# The execution action that is invoked periodically
		# former rtc_active_do()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onExecute(self, ec_id):
		self._robot._vmax=self._vmax[0]
		self._robot._vrate=self._vrate[0]
		self._robot._gripper_reverse=(self._gripper_reverse[0] == 1)
                self._robot.onExecute()
	
		return RTC.RTC_OK
	
	#	##
	#	#
	#	# The aborting action when main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The error action in ERROR state
	#	# former rtc_error_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK
	



def StdManipulatorInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=stdmanipulator_spec)
    manager.registerFactory(profile,
                            StdManipulator,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    StdManipulatorInit(manager)

    # Create a component
    comp = manager.createComponent("StdManipulator")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

