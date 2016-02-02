"""
Various utility methods that help the job of laying out the skeletton for a Rig.
"""
import pymel.core as pymel
from omtk.animation import mirrorPose
from omtk.libs import libPymel
from maya import OpenMaya
import math

def mirror_obj(obj_src, obj_dst=None):
    """
    Method to mirror joints in behavior.
    This use existing joint and doesn't break the skin or the network associated with the joints.
    """
    if obj_dst is None:
        obj_dst = mirrorPose.get_ctrl_friend(obj_src)
    if obj_src is obj_dst:
        return False
    tm = obj_src.getMatrix(worldSpace=True)
    new_tm = mirrorPose.mirror_matrix(tm, mirror_x=True, flip_rot_x=True, flip_rot_y=True, flip_rot_z=True)
    obj_dst.setMatrix(new_tm, worldSpace=True)
    return obj_dst

def transfer_rotation_to_joint_orient(obj):
    """
    In Maya it is not possible to do a "makeIdentity" command on a joint that is bound to a skinCluster.
    This method bypass this limitation.
    """
    mfn = obj.__apimfn__()
    rotation_orig = OpenMaya.MEulerRotation()
    mfn.getRotation(rotation_orig)
    rotation_xyz = rotation_orig.reorder(OpenMaya.MEulerRotation.kXYZ)
    obj.jointOrientX.set(math.degrees(rotation_xyz.x))
    obj.jointOrientY.set(math.degrees(rotation_xyz.y))
    obj.jointOrientZ.set(math.degrees(rotation_xyz.z))
    obj.rotateX.set(0)
    obj.rotateY.set(0)
    obj.rotateZ.set(0)


def mirror_jnt(obj_src, handle_joint_orient=True, create_missing=True):
    obj_dst = mirrorPose.get_ctrl_friend(obj_src)
    if obj_dst is None:
        src_name = obj_src.name()
        dst_name = mirrorPose.get_name_friend(src_name)
        if src_name == dst_name:
            return False

        obj_dst = pymel.createNode('joint')
        obj_dst.rename(dst_name)

        obj_src_parent = obj_src.getParent()
        if obj_src_parent:
            obj_dst_parent = mirrorPose.get_ctrl_friend(obj_src_parent)
            if obj_dst_parent:
                obj_dst.setParent(obj_dst_parent)

    mirror_obj(obj_src, obj_dst)
    if handle_joint_orient and isinstance(obj_dst, pymel.nodetypes.Joint):
        transfer_rotation_to_joint_orient(obj_dst)
    return obj_dst

def mirror_selected_joints():
    for obj in pymel.selected():
        mirror_obj(obj)

def mirror_jnts_l_to_r(**kwargs):
    jnts = sorted(pymel.ls('L_*_Jnt', type='joint') + pymel.ls('L_*_JEnd', type='joint'), key=libPymel.get_num_parents)
    for jnt in jnts:
        mirror_jnt(jnt, **kwargs)

def mirror_jnts_r_to_l(**kwargs):
    jnts = sorted(pymel.ls('R_*_Jnt', type='joint') + pymel.ls('R_*_JEnd', type='joint'), key=libPymel.get_num_parents)
    for jnt in jnts:
        mirror_jnt(jnt, **kwargs)
