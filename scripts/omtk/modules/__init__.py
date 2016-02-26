from omtk.libs import libPython
from omtk import classModule
from maya import cmds

# Import body modules
import rigFK
import rigIK
import rigRibbon
import rigSplineIK
import rigTwistbone
import rigLimb
import rigArm
import rigLeg
import rigHand
import rigDpSpine

# Import face modules
import rigFaceBrow
import rigFaceLids
import rigFaceLips

def _reload():
    # Reload body modules
    reload(rigFK)
    reload(rigIK)
    reload(rigRibbon)
    reload(rigSplineIK)
    reload(rigTwistbone)
    reload(rigLimb)
    reload(rigArm)
    reload(rigLeg)
    reload(rigHand)
    reload(rigDpSpine)

    # Reload face modules
    reload(rigFaceBrow)
    reload(rigFaceLids)
    reload(rigFaceLips)


def create(cls_name, *args, **kwargs):
    cls = libPython.get_class_def(cls_name, classModule.Module)
    if cls is None:
        raise Exception("Can't find any module named {0}".format(cls_name))
    cls(*args, **kwargs)
    return cls