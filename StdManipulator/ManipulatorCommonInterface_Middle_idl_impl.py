#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ManipulatorCommonInterface_Middle_idl_examplefile.py
 @brief Python example implementations generated from ManipulatorCommonInterface_Middle.idl
 @date $Date$


"""
#from __future__ import print_function
import omniORB
from omniORB import CORBA, PortableServer
import JARA_ARM, JARA_ARM__POA

from MySawyer import *

#
#
class ManipulatorCommonInterface_Middle_i (JARA_ARM__POA.ManipulatorCommonInterface_Middle):
    """
    @class ManipulatorCommonInterface_Middle_i
    Example class implementing IDL interface JARA_ARM.ManipulatorCommonInterface_Middle
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        #self._robot=MySawyer()
        self._robot=None

    # RETURN_ID closeGripper()
    def closeGripper(self):
        # Must return: result
        try:
          res=self._robot.closeGripper()
          if res:
            code=JARA_ARM.OK
            msg=''
          else:
            code=JARA_ARM.NG
            msg='Fail to close'
          return JARA_ARM.RETURN_ID(code, msg)
        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

       

    # RETURN_ID getBaseOffset(out HgMatrix offset)
    def getBaseOffset(self):
        # Must return: result, offset
        try:
          offset=self._robot.getBaseOffset()
          if offset is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), offset

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getFeedbackPosCartesian(out CarPosWithElbow pos)
    def getFeedbackPosCartesian(self):
        # Must return: result, pos
        try:
          pos=self._robot.getFeedbackPosCartesian()
          if pos is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), pos

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getMaxSpeedCartesian(out CartesianSpeed speed)
    def getMaxSpeedCartesian(self):
        # Must return: result, speed
        try:
          speed=self._robot.getMxSpeedCartesian()
          if speed is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), speed

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getMaxSpeedJoint(out DoubleSeq speed)
    def getMaxSpeedJoint(self):
        # Must return: result, speed
        try:
          speed=self._robot.getMxSpeedJoint()
          if speed is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), speed

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getMinAccelTimeCartesian(out double aclTime)
    def getMinAccelTimeCartesian(self):
        # Must return: result, aclTime
        try:
          aclTime=self._robot.getMinAccelTimeCartesian()
          if aclTime is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), aclTime

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getMinAccelTimeJoint(out double aclTime)
    def getMinAccelTimeJoint(self):
        # Must return: result, aclTime
        try:
          aclTime=self._robot.getMinAccelTimeJoint()
          if aclTime is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), aclTime

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getSoftLimitCartesian(out LimitValue xLimit, out LimitValue yLimit, out LimitValue zLimit)
    def getSoftLimitCartesian(self):
        # Must return: result, xLimit, yLimit, zLimit
        try:
          lmits=self._robot.getSoftLimitCartesian()
          if limits is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), limits[0],limits[1],limits[2]

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID moveGripper(in ULONG angleRatio)
    def moveGripper(self, angleRatio):
        # Must return: result
        try:
          res=self._robot.moveGripper()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID moveLinearCartesianAbs(in CarPosWithElbow carPoint)
    def moveLinearCartesianAbs(self, carPoint):
        # Must return: result
        try:
          res=self._robot.moveLinearCartesianAbs(carPoints)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID moveLinearCartesianRel(in CarPosWithElbow carPoint)
    def moveLinearCartesianRel(self, carPoint):
        # Must return: result
        try:
          res=self._robot.moveLinearCartesianRel(carPoints)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)


    # RETURN_ID movePTPCartesianAbs(in CarPosWithElbow carPoint)
    def movePTPCartesianAbs(self, carPoint):
        # Must return: result
        try:
          res=self._robot.movePTPCartesianAbs(carPoints)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID movePTPCartesianRel(in CarPosWithElbow carPoint)
    def movePTPCartesianRel(self, carPoint):
        # Must return: result
        try:
          res=self._robot.movePTPCartesianRel(carPoints)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID movePTPJointAbs(in JointPos jointPoints)
    def movePTPJointAbs(self, jointPoints):
        # Must return: result
        try:
          res=self._robot.movePTPJointAbs(jointPoints)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID movePTPJointRel(in JointPos jointPoints)
    def movePTPJointRel(self, jointPoints):
        # Must return: result
        try:
          res=self._robot.movePTPJointRel(jointPoints)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID openGripper()
    def openGripper(self):
        # Must return: result
        try:
          res=self._robot.openGripper()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID pause()
    def pause(self):
        # Must return: result
        try:
          res=self._robot.pause_motion()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID resume()
    def resume(self):
        # Must return: result
        try:
          res=self._robot.resume_motion()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID stop()
    def stop(self):
        # Must return: result
        try:
          res=self._robot.stop_motion()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setAccelTimeCartesian(in double aclTime)
    def setAccelTimeCartesian(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setAccelTimeCartesian(aclTime)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setAccelTimeJoint(in double aclTime)
    def setAccelTimeJoint(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setAccelTimeJoint(aclTime)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setBaseOffset(in HgMatrix offset)
    def setBaseOffset(self, offset):
        # Must return: result
        try:
          res=self._robot.setBaseOffset(offset)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setControlPointOffset(in HgMatrix offset)
    def setControlPointOffset(self, offset):
        # Must return: result
        try:
          res=self._robot.setControlPointOffset(offset)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setMaxSpeedCartesian(in CartesianSpeed speed)
    def setMaxSpeedCartesian(self, speed):
        # Must return: result
        try:
          res=self._robot.setMaxSpeedCartesian(speed)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setMaxSpeedJoint(in DoubleSeq speed)
    def setMaxSpeedJoint(self, speed):
        # Must return: result
        try:
          res=self._robot.setMaxSpeedJoint(speed)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setMinAccelTimeCartesian(in double aclTime)
    def setMinAccelTimeCartesian(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setMinAccelTimeCartesian(aclTime)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setMinAccelTimeJoint(in double aclTime)
    def setMinAccelTimeJoint(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setMinAccelTimeJoint(aclTime)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setSoftLimitCartesian(in LimitValue xLimit, in LimitValue yLimit, in LimitValue zLimit)
    def setSoftLimitCartesian(self, xLimit, yLimit, zLimit):
        # Must return: result
        try:
          res=self._robot.setSoftLimitCartesian(xLimt, yLimit, zLimit)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)
        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setSpeedCartesian(in ULONG spdRatio)
    def setSpeedCartesian(self, spdRatio):
        # Must return: result
        try:
          res=self._robot.setSpeedCartesian(spdRatio)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setSpeedJoint(in ULONG spdRatio)
    def setSpeedJoint(self, spdRatio):
        # Must return: result
        try:
          res=self._robot.setSpeedJoint(spdRatio)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID moveCircularCartesianAbs(in CarPosWithElbow carPointR, in CarPosWithElbow carPointT)
    def moveCircularCartesianAbs(self, carPointR, carPointT):
        # Must return: result
        try:
          res=self._robot.moveCircularCartesianAbs(carPointR, carPointT)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID moveCircularCartesianRel(in CarPosWithElbow carPointR, in CarPosWithElbow carPointT)
    def moveCircularCartesianRel(self, carPointR, carPointT):
        # Must return: result
        try:
          res=self._robot.moveCircularCartesianRel(carPointR, carPointT)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID setHome(in JointPos jointPoint)
    def setHome(self, jointPoint):
        # Must return: result
        try:
          res=self._robot.setHome(jointPoint)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID getHome(out JointPos jointPoint)
    def getHome(self):
        # Must return: result, jointPoint
        try:
          jointPoint=self._robot.getHome()
          if jointPoint is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), jointPoint

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

    # RETURN_ID goHome()
    def goHome(self):
        # Must return: result
        try:
          res=self._robot.goHome()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)


if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ManipulatorCommonInterface_Middle_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print ( orb.object_to_string(objref) )

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()

