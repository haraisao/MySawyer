#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file StdManipulator.py
 @brief Manipulator controller with JARA Standard Interfaces
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

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
		 ""]
# </rtc-template>
############################
#  Data Ports
rtc_dataports={}
rtc_dataports['joints']={'data_type':'RTC.TimedFloatSeq', 'direction':'in'}
rtc_dataports['grip']={'data_type':'RTC.TimedOctet', 'direction':'in'}
rtc_dataports['out_joints']={'data_type':'RTC.TimedFloatSeq', 'direction':'out'}
rtc_dataports['out_velocity']={'data_type':'RTC.TimedFloatSeq', 'direction':'out'}
rtc_dataports['out_torque']={'data_type':'RTC.TimedFloatSeq', 'direction':'out'}

# Service Ports
rtc_services={}
rtc_services['manipCommon']={'impl': ManipulatorCommonInterface_Common_i, 'direction':'provider',
                    'if_name': "JARA_ARM_ManipulatorCommonInterface_Common",
                    'if_type_name' :"JARA_ARM::ManipulatorCommonInterface_Common"}

rtc_services['manipMiddle']={'impl': ManipulatorCommonInterface_Middle_i, 'direction':'provider',
                    'if_name': "JARA_ARM_ManipulatorCommonInterface_Middle",
                    'if_type_name' :"JARA_ARM::ManipulatorCommonInterface_Middle"}

#  Parameters
rtc_params={}
rtc_params['vmax']={'__type__':'float', 'default':'0.3', '__widget__':'text'}
rtc_params['vrate']={'__type__':'float', 'default':'2.0', '__widget__':'text'}
rtc_params['accuracy']={'__type__':'float', 'default':'0.01', '__widget__': 'text'}
rtc_params['gripper_reverse']={'__type__':'int', 'default':'1', '__widget__':'radio', '__constraints__':'(0,1)'}


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

                for k in rtc_dataports.keys():
                  _d, _p=init_dataport(k, rtc_dataports[k]['data_type'], rtc_dataports[k]['direction'])
                  if rtc_dataports[k]['direction'] == 'in':
                    self.__dict__['_d_'+k] = _d
                    self.__dict__['_'+k+'In'] = _p
                  elif rtc_dataports[k]['direction'] == 'out':
                    self.__dict__['_d_'+k] = _d
                    self.__dict__['_'+k+'Out'] = _p
                  else:
                    pass


                for k in rtc_services.keys():
                  if rtc_services[k]['direction'] == 'provider':
		    self.__dict__['_'+k+'Port'] = OpenRTM_aist.CorbaPort(k)
		    self.__dict__['_'+k+'_service'] = rtc_services[k]['impl']()
                  if rtc_services[k]['direction'] == 'consumer':
		    self.__dict__['_'+k+'Port'] = OpenRTM_aist.CorbaPort(k)
		    self.__dict__['_'+k+'_service'] = OpenRTM_aist.CorbaConsumer(interfaceType=rtc_services[k]['if_type'])


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
                for x in init_params(rtc_params):
                    self.__dict__['_'+x[0]] = [x[1]]

		
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
                for k in rtc_params.keys():
          	  self.bindParameter(k, self.__dict__['_'+k], rtc_params[k]['default'])
                  
		
		# Set InPort buffers
                for k in rtc_dataports.keys():
                  if rtc_dataports[k]['direction'] == 'in':
		    self.addOutPort(k, self.__dict__['_'+k+'In'])
                  elif rtc_dataports[k]['direction'] == 'out':
		    self.addOutPort(k, self.__dict__['_'+k+'Out'])
                  else:
                    pass

		
		# Set service provider to Ports
                for k in rtc_services.keys():
                  if rtc_services[k]['direction'] == 'provider':
		    s_port=self.__dict__['_'+k+'Port']
		    service=self.__dict__['_'+k+'_service']
		    s_port.registerProvider(rtc_services[k]['if_name'], rtc_services[k]['if_type_name'], service)
		    self.addPort(s_port)
                  elif rtc_services[k]['direction'] == 'consumer':
		    s_port=self.__dict__['_'+k+'Port']
		    service=self.__dict__['_'+k+'_service']
		    s_port.registerConsumer(rtc_services[k]['if_name'], rtc_services[k]['if_type_name'], service)
		    self.addPort(s_port)

		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		
		return RTC.RTC_OK
	
	def onActivated(self, ec_id):
                self._robot=MySawyer(self.getInstanceName(), anonymous=False)
                self._robot.activate()
                self._robot._is_running=True
		self._manipCommon_service._robot=self._robot
		self._manipMiddle_service._robot=self._robot
		#self._JARA_ARM_ManipulatorCommonInterface_Common._robot=self._robot
		#self._JARA_ARM_ManipulatorCommonInterface_Middle._robot=self._robot
	
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
	

#########################################
#
def init_params_spec(spec, param):
  for k1 in param.keys():
    for k2 in param[k1].keys():
      e="conf." + k2 + "." + k1
      v=param[k1][k2]
      spec.insert(-1, e)
      spec.insert(-1, v)

def init_params(param):
  res=[]
  for k1 in param.keys():
    if param[k1]['__type__'] == 'string':
      val=param[k1]['default']
    else:
      val=eval(param[k1]['default'])
    res.append([k1, val])
  return res

def init_dataport(name, dname, typ):
  m, d=dname.split('.')
  d_type = eval(m+"._d_"+d)
  v_arg = [None] * ((len(d_type) - 4) / 2)
  _d = eval(dname)(*v_arg)
  if typ == 'in':
    _p = OpenRTM_aist.InPort(name, _d)
  elif typ == 'out':
    _p = OpenRTM_aist.OutPort(name, _d)
  else:
    _p = None

  return _d,_p

#########################################

def StdManipulatorInit(manager):
    init_params_spec(stdmanipulator_spec, rtc_params)
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

