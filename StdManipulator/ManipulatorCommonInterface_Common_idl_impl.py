#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ManipulatorCommonInterface_Common_idl_examplefile.py
 @brief Python example implementations generated from ManipulatorCommonInterface_Common.idl
 @date $Date$


"""
#from __future__ import print_function
import omniORB
from omniORB import CORBA, PortableServer
import JARA_ARM, JARA_ARM__POA

from MySawyer import *

#
#
def mk_return_id(code, msg=''):
  return JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.RETURN_ID(code,msg)


#
#
class ManipulatorCommonInterface_Common_i (JARA_ARM__POA.ManipulatorCommonInterface_Common):
    """
    @class ManipulatorCommonInterface_Common_i
    Example class implementing IDL interface JARA_ARM.ManipulatorCommonInterface_Common
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        #self._robot=MySawyer()
        self._robot=None

    # RETURN_ID clearAlarms()
    def clearAlarms(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result
        try:
          res=self._robot.clearAlarms()
          if res is None:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          return mk_return_id(code)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getActiveAlarm(out AlarmSeq alarms)
    def getActiveAlarm(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, alarms
        try:
          res=self._robot.getActiveAlarm()
          if res is None:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          return mk_return_id(code), res

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getFeedbackPosJoint(out JointPos pos)
    def getFeedbackPosJoint(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, pos
        try:
          pos=self._robot.getFeedbackPosJoint()
          if pos is None :
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          return mk_return_id(code),pos

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getManipInfo(out ManipInfo mInfo)
    def getManipInfo(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, mInfo
        try:
          mInfo=self._robot.getManipInto()
          if mInfo is None :
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          return mk_return_id(code),mInfo

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getSoftLimitJoint(out LimitSeq softLimit)
    def getSoftLimitJoint(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, softLimit
        try:
          softLimit=self._robot.getSoftLimitJoint()
          if softLimit is None :
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          return mk_return_id(code),softLimit

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getState(out ULONG state)
    def getState(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, state
        try:
          state=self._robot.getState()
          if state is None :
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          return mk_return_id(code),state

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID servoOFF()
    def servoOFF(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result
        try:
          res=self._robot.servoOFF()
          if res is None :
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          return mk_return_id(code)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID servoON()
    def servoON(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result
        try:
          res=self._robot.servoON()
          if res is None :
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          return mk_return_id(code)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setSoftLimitJoint(in LimitSeq softLimit)
    def setSoftLimitJoint(self, softLimit):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result
        try:
          res=self._robot.setSoftLimitJoint(softLimit)
          if res is None :
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.NG
          else:
            code=JARA_ARM.ManipulatorCommonInterface_DataTypes_idl._0_JARA_ARM.OK
          return mk_return_id(code)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)



if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ManipulatorCommonInterface_Common_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print(orb.object_to_string(objref))

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()

