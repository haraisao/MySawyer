# Python stubs generated by omniidl from idl/ManipulatorCommonInterface_Middle.idl
# DO NOT EDIT THIS FILE!

import omniORB, _omnipy
from omniORB import CORBA, PortableServer
_0_CORBA = CORBA


_omnipy.checkVersion(4,2, __file__, 1)

try:
    property
except NameError:
    def property(*args):
        return None


# #include "BasicDataType.idl"
import BasicDataType_idl
_0_RTC = omniORB.openModule("RTC")
_0_RTC__POA = omniORB.openModule("RTC__POA")

# #include "ManipulatorCommonInterface_DataTypes.idl"
import ManipulatorCommonInterface_DataTypes_idl
_0_JARA_ARM = omniORB.openModule("JARA_ARM")
_0_JARA_ARM__POA = omniORB.openModule("JARA_ARM__POA")

#
# Start of module "JARA_ARM"
#
__name__ = "JARA_ARM"
_0_JARA_ARM = omniORB.openModule("JARA_ARM", r"idl/ManipulatorCommonInterface_Middle.idl")
_0_JARA_ARM__POA = omniORB.openModule("JARA_ARM__POA", r"idl/ManipulatorCommonInterface_Middle.idl")


# typedef ... HgMatrix
class HgMatrix:
    _NP_RepositoryId = "IDL:JARA_ARM/HgMatrix:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_JARA_ARM.HgMatrix = HgMatrix
_0_JARA_ARM._d_HgMatrix  = (omniORB.tcInternal.tv_array, (omniORB.tcInternal.tv_array, omniORB.tcInternal.tv_double, 4), 3)
_0_JARA_ARM._ad_HgMatrix = (omniORB.tcInternal.tv_alias, HgMatrix._NP_RepositoryId, "HgMatrix", (omniORB.tcInternal.tv_array, (omniORB.tcInternal.tv_array, omniORB.tcInternal.tv_double, 4), 3))
_0_JARA_ARM._tc_HgMatrix = omniORB.tcInternal.createTypeCode(_0_JARA_ARM._ad_HgMatrix)
omniORB.registerType(HgMatrix._NP_RepositoryId, _0_JARA_ARM._ad_HgMatrix, _0_JARA_ARM._tc_HgMatrix)
del HgMatrix

# struct CarPosWithElbow
_0_JARA_ARM.CarPosWithElbow = omniORB.newEmptyClass()
class CarPosWithElbow (omniORB.StructBase):
    _NP_RepositoryId = "IDL:JARA_ARM/CarPosWithElbow:1.0"

    def __init__(self, carPos, elbow, structFlag):
        self.carPos = carPos
        self.elbow = elbow
        self.structFlag = structFlag

_0_JARA_ARM.CarPosWithElbow = CarPosWithElbow
_0_JARA_ARM._d_CarPosWithElbow  = (omniORB.tcInternal.tv_struct, CarPosWithElbow, CarPosWithElbow._NP_RepositoryId, "CarPosWithElbow", "carPos", omniORB.typeMapping["IDL:JARA_ARM/HgMatrix:1.0"], "elbow", omniORB.tcInternal.tv_double, "structFlag", omniORB.typeMapping["IDL:JARA_ARM/ULONG:1.0"])
_0_JARA_ARM._tc_CarPosWithElbow = omniORB.tcInternal.createTypeCode(_0_JARA_ARM._d_CarPosWithElbow)
omniORB.registerType(CarPosWithElbow._NP_RepositoryId, _0_JARA_ARM._d_CarPosWithElbow, _0_JARA_ARM._tc_CarPosWithElbow)
del CarPosWithElbow

# struct CartesianSpeed
_0_JARA_ARM.CartesianSpeed = omniORB.newEmptyClass()
class CartesianSpeed (omniORB.StructBase):
    _NP_RepositoryId = "IDL:JARA_ARM/CartesianSpeed:1.0"

    def __init__(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

_0_JARA_ARM.CartesianSpeed = CartesianSpeed
_0_JARA_ARM._d_CartesianSpeed  = (omniORB.tcInternal.tv_struct, CartesianSpeed, CartesianSpeed._NP_RepositoryId, "CartesianSpeed", "translation", omniORB.tcInternal.tv_double, "rotation", omniORB.tcInternal.tv_double)
_0_JARA_ARM._tc_CartesianSpeed = omniORB.tcInternal.createTypeCode(_0_JARA_ARM._d_CartesianSpeed)
omniORB.registerType(CartesianSpeed._NP_RepositoryId, _0_JARA_ARM._d_CartesianSpeed, _0_JARA_ARM._tc_CartesianSpeed)
del CartesianSpeed

# interface ManipulatorCommonInterface_Middle
_0_JARA_ARM._d_ManipulatorCommonInterface_Middle = (omniORB.tcInternal.tv_objref, "IDL:JARA_ARM/ManipulatorCommonInterface_Middle:1.0", "ManipulatorCommonInterface_Middle")
omniORB.typeMapping["IDL:JARA_ARM/ManipulatorCommonInterface_Middle:1.0"] = _0_JARA_ARM._d_ManipulatorCommonInterface_Middle
_0_JARA_ARM.ManipulatorCommonInterface_Middle = omniORB.newEmptyClass()
class ManipulatorCommonInterface_Middle :
    _NP_RepositoryId = _0_JARA_ARM._d_ManipulatorCommonInterface_Middle[1]

    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")

    _nil = CORBA.Object._nil


_0_JARA_ARM.ManipulatorCommonInterface_Middle = ManipulatorCommonInterface_Middle
_0_JARA_ARM._tc_ManipulatorCommonInterface_Middle = omniORB.tcInternal.createTypeCode(_0_JARA_ARM._d_ManipulatorCommonInterface_Middle)
omniORB.registerType(ManipulatorCommonInterface_Middle._NP_RepositoryId, _0_JARA_ARM._d_ManipulatorCommonInterface_Middle, _0_JARA_ARM._tc_ManipulatorCommonInterface_Middle)

# ManipulatorCommonInterface_Middle operations and attributes
ManipulatorCommonInterface_Middle._d_closeGripper = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_getBaseOffset = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.typeMapping["IDL:JARA_ARM/HgMatrix:1.0"]), None)
ManipulatorCommonInterface_Middle._d_getFeedbackPosCartesian = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"]), None)
ManipulatorCommonInterface_Middle._d_getMaxSpeedCartesian = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.typeMapping["IDL:JARA_ARM/CartesianSpeed:1.0"]), None)
ManipulatorCommonInterface_Middle._d_getMaxSpeedJoint = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.typeMapping["IDL:JARA_ARM/DoubleSeq:1.0"]), None)
ManipulatorCommonInterface_Middle._d_getMinAccelTimeCartesian = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.tcInternal.tv_double), None)
ManipulatorCommonInterface_Middle._d_getMinAccelTimeJoint = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.tcInternal.tv_double), None)
ManipulatorCommonInterface_Middle._d_getSoftLimitCartesian = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.typeMapping["IDL:JARA_ARM/LimitValue:1.0"], omniORB.typeMapping["IDL:JARA_ARM/LimitValue:1.0"], omniORB.typeMapping["IDL:JARA_ARM/LimitValue:1.0"]), None)
ManipulatorCommonInterface_Middle._d_moveGripper = ((omniORB.typeMapping["IDL:JARA_ARM/ULONG:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_moveLinearCartesianAbs = ((omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_moveLinearCartesianRel = ((omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_movePTPCartesianAbs = ((omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_movePTPCartesianRel = ((omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_movePTPJointAbs = ((omniORB.typeMapping["IDL:JARA_ARM/JointPos:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_movePTPJointRel = ((omniORB.typeMapping["IDL:JARA_ARM/JointPos:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_openGripper = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_pause = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_resume = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_stop = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setAccelTimeCartesian = ((omniORB.tcInternal.tv_double, ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setAccelTimeJoint = ((omniORB.tcInternal.tv_double, ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setBaseOffset = ((omniORB.typeMapping["IDL:JARA_ARM/HgMatrix:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setControlPointOffset = ((omniORB.typeMapping["IDL:JARA_ARM/HgMatrix:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setMaxSpeedCartesian = ((omniORB.typeMapping["IDL:JARA_ARM/CartesianSpeed:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setMaxSpeedJoint = ((omniORB.typeMapping["IDL:JARA_ARM/DoubleSeq:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setMinAccelTimeCartesian = ((omniORB.tcInternal.tv_double, ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setMinAccelTimeJoint = ((omniORB.tcInternal.tv_double, ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setSoftLimitCartesian = ((omniORB.typeMapping["IDL:JARA_ARM/LimitValue:1.0"], omniORB.typeMapping["IDL:JARA_ARM/LimitValue:1.0"], omniORB.typeMapping["IDL:JARA_ARM/LimitValue:1.0"]), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setSpeedCartesian = ((omniORB.typeMapping["IDL:JARA_ARM/ULONG:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setSpeedJoint = ((omniORB.typeMapping["IDL:JARA_ARM/ULONG:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_moveCircularCartesianAbs = ((omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"], omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"]), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_moveCircularCartesianRel = ((omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"], omniORB.typeMapping["IDL:JARA_ARM/CarPosWithElbow:1.0"]), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_setHome = ((omniORB.typeMapping["IDL:JARA_ARM/JointPos:1.0"], ), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)
ManipulatorCommonInterface_Middle._d_getHome = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], omniORB.typeMapping["IDL:JARA_ARM/JointPos:1.0"]), None)
ManipulatorCommonInterface_Middle._d_goHome = ((), (omniORB.typeMapping["IDL:JARA_ARM/RETURN_ID:1.0"], ), None)

# ManipulatorCommonInterface_Middle object reference
class _objref_ManipulatorCommonInterface_Middle (CORBA.Object):
    _NP_RepositoryId = ManipulatorCommonInterface_Middle._NP_RepositoryId

    def __init__(self, obj):
        CORBA.Object.__init__(self, obj)

    def closeGripper(self, *args):
        return self._obj.invoke("closeGripper", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_closeGripper, args)

    def getBaseOffset(self, *args):
        return self._obj.invoke("getBaseOffset", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getBaseOffset, args)

    def getFeedbackPosCartesian(self, *args):
        return self._obj.invoke("getFeedbackPosCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getFeedbackPosCartesian, args)

    def getMaxSpeedCartesian(self, *args):
        return self._obj.invoke("getMaxSpeedCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMaxSpeedCartesian, args)

    def getMaxSpeedJoint(self, *args):
        return self._obj.invoke("getMaxSpeedJoint", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMaxSpeedJoint, args)

    def getMinAccelTimeCartesian(self, *args):
        return self._obj.invoke("getMinAccelTimeCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMinAccelTimeCartesian, args)

    def getMinAccelTimeJoint(self, *args):
        return self._obj.invoke("getMinAccelTimeJoint", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMinAccelTimeJoint, args)

    def getSoftLimitCartesian(self, *args):
        return self._obj.invoke("getSoftLimitCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getSoftLimitCartesian, args)

    def moveGripper(self, *args):
        return self._obj.invoke("moveGripper", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveGripper, args)

    def moveLinearCartesianAbs(self, *args):
        return self._obj.invoke("moveLinearCartesianAbs", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveLinearCartesianAbs, args)

    def moveLinearCartesianRel(self, *args):
        return self._obj.invoke("moveLinearCartesianRel", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveLinearCartesianRel, args)

    def movePTPCartesianAbs(self, *args):
        return self._obj.invoke("movePTPCartesianAbs", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPCartesianAbs, args)

    def movePTPCartesianRel(self, *args):
        return self._obj.invoke("movePTPCartesianRel", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPCartesianRel, args)

    def movePTPJointAbs(self, *args):
        return self._obj.invoke("movePTPJointAbs", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPJointAbs, args)

    def movePTPJointRel(self, *args):
        return self._obj.invoke("movePTPJointRel", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPJointRel, args)

    def openGripper(self, *args):
        return self._obj.invoke("openGripper", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_openGripper, args)

    def pause(self, *args):
        return self._obj.invoke("pause", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_pause, args)

    def resume(self, *args):
        return self._obj.invoke("resume", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_resume, args)

    def stop(self, *args):
        return self._obj.invoke("stop", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_stop, args)

    def setAccelTimeCartesian(self, *args):
        return self._obj.invoke("setAccelTimeCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setAccelTimeCartesian, args)

    def setAccelTimeJoint(self, *args):
        return self._obj.invoke("setAccelTimeJoint", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setAccelTimeJoint, args)

    def setBaseOffset(self, *args):
        return self._obj.invoke("setBaseOffset", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setBaseOffset, args)

    def setControlPointOffset(self, *args):
        return self._obj.invoke("setControlPointOffset", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setControlPointOffset, args)

    def setMaxSpeedCartesian(self, *args):
        return self._obj.invoke("setMaxSpeedCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMaxSpeedCartesian, args)

    def setMaxSpeedJoint(self, *args):
        return self._obj.invoke("setMaxSpeedJoint", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMaxSpeedJoint, args)

    def setMinAccelTimeCartesian(self, *args):
        return self._obj.invoke("setMinAccelTimeCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMinAccelTimeCartesian, args)

    def setMinAccelTimeJoint(self, *args):
        return self._obj.invoke("setMinAccelTimeJoint", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMinAccelTimeJoint, args)

    def setSoftLimitCartesian(self, *args):
        return self._obj.invoke("setSoftLimitCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setSoftLimitCartesian, args)

    def setSpeedCartesian(self, *args):
        return self._obj.invoke("setSpeedCartesian", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setSpeedCartesian, args)

    def setSpeedJoint(self, *args):
        return self._obj.invoke("setSpeedJoint", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setSpeedJoint, args)

    def moveCircularCartesianAbs(self, *args):
        return self._obj.invoke("moveCircularCartesianAbs", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveCircularCartesianAbs, args)

    def moveCircularCartesianRel(self, *args):
        return self._obj.invoke("moveCircularCartesianRel", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveCircularCartesianRel, args)

    def setHome(self, *args):
        return self._obj.invoke("setHome", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setHome, args)

    def getHome(self, *args):
        return self._obj.invoke("getHome", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getHome, args)

    def goHome(self, *args):
        return self._obj.invoke("goHome", _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_goHome, args)

omniORB.registerObjref(ManipulatorCommonInterface_Middle._NP_RepositoryId, _objref_ManipulatorCommonInterface_Middle)
_0_JARA_ARM._objref_ManipulatorCommonInterface_Middle = _objref_ManipulatorCommonInterface_Middle
del ManipulatorCommonInterface_Middle, _objref_ManipulatorCommonInterface_Middle

# ManipulatorCommonInterface_Middle skeleton
__name__ = "JARA_ARM__POA"
class ManipulatorCommonInterface_Middle (PortableServer.Servant):
    _NP_RepositoryId = _0_JARA_ARM.ManipulatorCommonInterface_Middle._NP_RepositoryId


    _omni_op_d = {"closeGripper": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_closeGripper, "getBaseOffset": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getBaseOffset, "getFeedbackPosCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getFeedbackPosCartesian, "getMaxSpeedCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMaxSpeedCartesian, "getMaxSpeedJoint": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMaxSpeedJoint, "getMinAccelTimeCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMinAccelTimeCartesian, "getMinAccelTimeJoint": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getMinAccelTimeJoint, "getSoftLimitCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getSoftLimitCartesian, "moveGripper": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveGripper, "moveLinearCartesianAbs": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveLinearCartesianAbs, "moveLinearCartesianRel": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveLinearCartesianRel, "movePTPCartesianAbs": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPCartesianAbs, "movePTPCartesianRel": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPCartesianRel, "movePTPJointAbs": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPJointAbs, "movePTPJointRel": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_movePTPJointRel, "openGripper": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_openGripper, "pause": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_pause, "resume": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_resume, "stop": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_stop, "setAccelTimeCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setAccelTimeCartesian, "setAccelTimeJoint": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setAccelTimeJoint, "setBaseOffset": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setBaseOffset, "setControlPointOffset": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setControlPointOffset, "setMaxSpeedCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMaxSpeedCartesian, "setMaxSpeedJoint": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMaxSpeedJoint, "setMinAccelTimeCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMinAccelTimeCartesian, "setMinAccelTimeJoint": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setMinAccelTimeJoint, "setSoftLimitCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setSoftLimitCartesian, "setSpeedCartesian": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setSpeedCartesian, "setSpeedJoint": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setSpeedJoint, "moveCircularCartesianAbs": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveCircularCartesianAbs, "moveCircularCartesianRel": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_moveCircularCartesianRel, "setHome": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_setHome, "getHome": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_getHome, "goHome": _0_JARA_ARM.ManipulatorCommonInterface_Middle._d_goHome}

ManipulatorCommonInterface_Middle._omni_skeleton = ManipulatorCommonInterface_Middle
_0_JARA_ARM__POA.ManipulatorCommonInterface_Middle = ManipulatorCommonInterface_Middle
omniORB.registerSkeleton(ManipulatorCommonInterface_Middle._NP_RepositoryId, ManipulatorCommonInterface_Middle)
del ManipulatorCommonInterface_Middle
__name__ = "JARA_ARM"

#
# End of module "JARA_ARM"
#
__name__ = "ManipulatorCommonInterface_Middle_idl"

_exported_modules = ( "JARA_ARM", )

# The end.
