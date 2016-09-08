import itertools
import logging

import pymel.core as pymel

from omtk.core import classModule
from omtk.core import classCtrl
from omtk.libs import libPymel
from omtk.libs import libPython
from omtk.libs import libRigging
from omtk.libs import libCtrlShapes
from omtk.libs.libRigging import get_average_pos_between_nodes
from omtk.modules import rigFaceAvar

log = logging.getLogger('omtk')


class CtrlFaceUpp(rigFaceAvar.BaseCtrlFace):
    def __createNode__(self, **kwargs):
        return libCtrlShapes.create_triangle_upp()

class CtrlFaceLow(rigFaceAvar.BaseCtrlFace):
    def __createNode__(self, **kwargs):
        return libCtrlShapes.create_triangle_low()

class CtrlFaceAll(rigFaceAvar.BaseCtrlFace):
    def __createNode__(self, **kwargs):
        # todo: find the best shape
        transform, _ = libCtrlShapes.create_shape_circle()
        return transform

class CtrlFaceHorizontal(rigFaceAvar.BaseCtrlFace):
    def __createNode__(self, **kwargs):
        return libCtrlShapes.create_triangle_left()

def _find_mid_avar(avars):
    jnts = [avar.jnt for avar in avars]
    nearest_jnt = get_average_pos_between_nodes(jnts)
    return avars[jnts.index(nearest_jnt)] if nearest_jnt else None

class AvarGrp(rigFaceAvar.AbstractAvar):
    """
    Base class for a group of 'avars' that share similar properties.
    Also global avars will be provided to controll all avars.
    """
    _CLS_AVAR = rigFaceAvar.AvarSimple
    _CLS_AVAR_ALL = rigFaceAvar.AvarFollicleGlobal
    SHOW_IN_UI = True

    # Disable if the AvarGrp don't need any geometry to function.
    # This is mainly a workaround a limitation of the design which doesn't allow access to the avars without building.
    VALIDATE_MESH = True

    # Enable this flag if the module contain only one influence.
    # ex: The FaceJaw module can accept two objects. The jaw and the jaw_end. However we consider the jaw_end as extra information for the positioning.
    # TODO: Find a generic way to get the InteractiveCtrl follicle position.
    SINGLE_INFLUENCE = False

    #
    # Influences properties
    #

    @property
    def jnt_inn(self):
        # TODO: Find a better way
        return self.jnts[0]

    @property
    def jnt_mid(self):
        # TODO: Find a better way
        i = (len(self.jnts)-1) / 2
        return self.jnts[i]

    @property
    def jnt_out(self):
        # TODO: Find a better way
        return self.jnts[-1]

    @libPython.cached_property()
    def jnts_upp(self):
        # TODO: Find a better way
        fnFilter = lambda jnt: 'upp' in jnt.name().lower()
        return filter(fnFilter, self.jnts)

    @libPython.cached_property()
    def jnt_upp_mid(self):
        return get_average_pos_between_nodes(self.jnts_upp)

    @libPython.cached_property()
    def jnts_low(self):
        # TODO: Find a better way
        fnFilter = lambda jnt: 'low' in jnt.name().lower()
        return filter(fnFilter, self.jnts)

    @libPython.cached_property()
    def jnt_low_mid(self):
        return get_average_pos_between_nodes(self.jnts_low)

    @libPython.cached_property()
    def jnts_l(self):
        middle = libRigging.get_average_pos_between_vectors(self.jnts)
        fn_filter = lambda jnt: jnt.getTranslation(space='world').x >= middle.x
        return filter(fn_filter, self.jnts)

    @libPython.cached_property()
    def jnts_r(self):
        middle = libRigging.get_average_pos_between_vectors(self.jnts)
        fn_filter = lambda jnt: jnt.getTranslation(space='world').x < middle.x
        return filter(fn_filter, self.jnts)

    @libPython.cached_property()
    def jnt_l_mid(self):
        """
        :return: The left most joint (highest positive distance in x)
        """
        fn_get_pos_x = lambda x: x.getTranslation(space='world').x
        return next(iter(reversed(sorted(self.jnts_l, key=fn_get_pos_x))), None)

    @libPython.cached_property()
    def jnt_r_mid(self):
        """
        :return: The right most joint (highest negative distance in x)
        """
        fn_get_pos_x = lambda x: x.getTranslation(space='world').x
        return next(iter(sorted(self.jnts_r, key=fn_get_pos_x)), None)

    #
    # Avar properties
    # Note that theses are only accessible after the avars have been built.
    #

    @property  # Note that since the avars are volatile we don't want to cache this property.
    def avars_upp(self):
        # TODO: Find a better way
        fnFilter = lambda avar: 'upp' in avar.name.lower()
        return filter(fnFilter, self.avars)

    @property  # Note that since the avars are volatile we don't want to cache this property.
    def avars_low(self):
        # TODO: Find a better way
        fnFilter = lambda avar: 'low' in avar.name.lower()
        return filter(fnFilter, self.avars)

    @property
    def avar_upp_mid(self):
        return _find_mid_avar(self.avars_upp)

    @property
    def avar_low_mid(self):
        return _find_mid_avar(self.avars_low)

    @property
    def avar_inn(self):
        return self.avars[0] if self.avars else None

    @property
    def avar_mid(self):
        return _find_mid_avar(self.avars)

    @property
    def avar_out(self):
        return self.avars[-1] if self.avars else None

    @libPython.cached_property()
    def avars_l(self):
        """
        There are two ways to separate avars in left and right side
        :return:
        """
        middle = libRigging.get_average_pos_between_vectors(self.jnts)
        fn_filter = lambda avar: avar.jnt.getTranslation(space='world').x >= middle.x
        return filter(fn_filter, self.avars)

    @libPython.cached_property()
    def avars_r(self):
        middle = libRigging.get_average_pos_between_vectors(self.jnts)
        fn_filter = lambda avar: avar.jnt.getTranslation(space='world').x < middle.x
        return filter(fn_filter, self.avars)

    @libPython.cached_property()
    def avar_l_mid(self):
        fn_get_avar_pos_x = lambda avar: avar.jnt.getTranslation(space='world').x
        return next(iter(reversed(sorted(self.avars_l, key=fn_get_avar_pos_x))), None)

    @libPython.cached_property()
    def avar_r_mid(self):
        fn_get_avar_pos_x = lambda avar: avar.jnt.getTranslation(space='world').x
        return next(iter(sorted(self.avars_l, key=fn_get_avar_pos_x)), None)

    #
    #
    #

    def __init__(self, *args, **kwargs):
        super(AvarGrp, self).__init__(*args, **kwargs)
        self.avars = []
        self.preDeform = False

    @libPython.cached_property()
    def jnts(self):
        fn_is_nurbsSurface = lambda obj: libPymel.isinstance_of_transform(obj, pymel.nodetypes.Joint)
        return filter(fn_is_nurbsSurface, self.input)

    def connect_global_avars(self):
        for avar in self.avars:
            libRigging.connectAttr_withBlendWeighted(self.attr_ud, avar.attr_ud)
            libRigging.connectAttr_withBlendWeighted(self.attr_lr, avar.attr_lr)
            libRigging.connectAttr_withBlendWeighted(self.attr_fb, avar.attr_fb)
            libRigging.connectAttr_withBlendWeighted(self.attr_yw, avar.attr_yw)
            libRigging.connectAttr_withBlendWeighted(self.attr_pt, avar.attr_pt)
            libRigging.connectAttr_withBlendWeighted(self.attr_rl, avar.attr_rl)

    def get_multiplier_u(self):
        return 1.0

    def get_multiplier_v(self):
        return 1.0

    def _get_default_ctrl_size(self, rig):
        """
        Resolve the desired ctrl size
        One thing we are sure is that ctrls should not overlay,
        so we'll max out their radius to half of the shortest distances between each.
        Also the radius cannot be bigger than 3% of the head length.
        """
        ctrl_size = 1
        EPSILON = 0.001 # prevent ctrl from dissapearing if two influences share the same location
        max_ctrl_size = None

        # Resolve maximum ctrl size from head joint
        try:
            head_length = rig.get_head_length()
        except Exception, e:
            head_length = None
            log.warning(e)
        if head_length:
            max_ctrl_size = rig.get_head_length() * 0.03

        if len(self.jnts) > 1:
            new_ctrl_size = min(libPymel.distance_between_nodes(jnt_src, jnt_dst) for jnt_src, jnt_dst in itertools.permutations(self.jnts, 2)) / 2.0
            if new_ctrl_size > EPSILON:
                ctrl_size = new_ctrl_size
            
            if max_ctrl_size is not None and ctrl_size > max_ctrl_size:
                log.warning("Limiting ctrl size to {0}".format(max_ctrl_size))
                ctrl_size = max_ctrl_size
        else:
            log.warning("Can't automatically resolve ctrl size, using default {0}".format(ctrl_size))

        return ctrl_size

    def _get_avars_influences(self):
        """
        Return the influences that need to have avars associated with.
        Normally for 3 influences, we create 3 avars.
        However if the SINGLE_INFLUENCE flag is up, only the first influence will be rigged, the others
        mights be handled upstream. (ex: FaceJaw).
        """
        if self.SINGLE_INFLUENCE:
            return [self.jnt]
        else:
            return self.jnts

    def validate(self, rig):
        """
        Ensure all influences are influencing a geometry.
        This allow us to prevent the user to find out when building.
        """
        super(AvarGrp, self).validate(rig)

        if self.VALIDATE_MESH:
            avar_influences = self._get_avars_influences()
            for jnt in avar_influences:
                mesh = rig.get_farest_affected_mesh(jnt)
                if not mesh:
                    raise Exception("Can't find mesh affected by {0}.".format(jnt))

        # Try to resolve the head joint.
        # With strict=True, an exception will be raised if nothing is found.
        rig.get_head_jnt(strict=True)

    def _need_to_define_avars(self):
        """
        Check if we need to reset the property containing the avars associated with the influences.
        It some rare cases it might be necessary to reset everything, however this would be considered a last-case
        scenario since this could have unintended consequences as loosing any held information (like ctrl shapes).
        """
        # First build
        if not self.avars:
            return True

        # If the influence and avars count mismatch, we need to rebuild everything.
        # Also if the desired avars type have changed, we need to rebuild everything.
        avar_influences = self._get_avars_influences()
        if len(filter(lambda x: isinstance(x, self._CLS_AVAR), self.avars)) != len(avar_influences):
            log.warning("Mismatch between avars and jnts tables. Will reset the avars table.")
            return True

        return False

    def _define_avars(self, rig):
        """
        For each influence, create it's associated avar instance.
        """
        avars = []
        avar_influences = self._get_avars_influences()
        # Connect global avars to invidial avars
        for jnt in avar_influences:
            avar = self.create_avar(rig, jnt)
            avars.append(avar)
        return avars

    def _build_avars(self, rig, create_ctrls=True, constraint=True, connect_global_scale=None, **kwargs):
        """
        Call the .build() method on all avars.
        """
        ctrl_size = self._get_default_ctrl_size(rig)

        # Resolve the U and V modifiers.
        # Note that this only applies to avars on a surface.
        # TODO: Move to AvarGrpOnSurface
        mult_u = self.get_multiplier_u()
        mult_v = self.get_multiplier_v()

        # Build avars and connect them to global avars
        avar_influences = self._get_avars_influences()
        for jnt, avar in zip(avar_influences, self.avars):
            self.configure_avar(rig, avar)

            # HACK: Set module name using rig nomenclature.
            # TODO: Do this in the back-end
            avar.name = rig.nomenclature(jnt.name()).resolve()

            # HACK: Validate avars at runtime
            # TODO: Find a way to validate before build without using VALIDATE_MESH
            try:
                avar.validate(rig)
            except Exception, e:
                log.warning("Can't build avar {0}, failed validation: {1}".format(
                    avar.name,
                    e
                ))
                continue

            avar.build(rig,
                       create_ctrl=create_ctrls,
                       constraint=constraint,
                       ctrl_size=ctrl_size,
                       mult_u=mult_u,
                       mult_v=mult_v,
                       connect_global_scale=connect_global_scale,
                       **kwargs)
            if avar.grp_anm:
                avar.grp_anm.setParent(self.grp_anm)
            avar.grp_rig.setParent(self.grp_rig)

    def build(self, rig, connect_global_scale=None, create_ctrls=True, parent=None, constraint=True, **kwargs):
        if parent is None:
            parent = not self.preDeform

        if connect_global_scale is None:
            connect_global_scale = self.preDeform

        super(AvarGrp, self).build(rig, connect_global_scale=connect_global_scale, parent=parent, **kwargs)

        # Create avars if needed (this will get skipped if the module have already been built once)
        if self._need_to_define_avars():
            self.avars = self._define_avars(rig)

        self._build_avars(rig, connect_global_scale=connect_global_scale, create_ctrls=create_ctrls, constraint=constraint, **kwargs)

        self.connect_global_avars()

        # If the deformation order is set to post (aka the deformer is in the final skinCluster)
        # we will want the offset node to follow it's original parent (ex: the head)
        if parent and self.parent:
            for avar in self.avars:
                layer_offset = avar._stack._layers[0]
                pymel.parentConstraint(self.parent, layer_offset, maintainOffset=True)
                pymel.scaleConstraint(self.parent, layer_offset, maintainOffset=True)

    def unbuild(self):
        for avar in self.avars:
            avar.unbuild()
        super(AvarGrp, self).unbuild()

    def get_ctrls(self, **kwargs):
        for ctrl in super(AvarGrp, self).get_ctrls(**kwargs):
            yield ctrl
        for avar in self.avars:
            for ctrl in avar.get_ctrls():
                yield ctrl

    @classModule.decorator_uiexpose
    def calibrate(self, rig):
        for avar in self.avars:
            avar.calibrate()

    def create_abstract_avar(self, rig, cls, ref, name=None, cls_avar=None, **kwargs):
        """
        Factory method to create abstract avars that will controller other avars.
        """
        if cls_avar is None:
            cls_avar = rigFaceAvar.AvarSimple

        input = filter(None, [ref, self.surface])

        avar = cls_avar(input, name=name)  # TODO: Replace by Abstract Avar
        avar._CLS_CTRL = cls

        return avar

    def create_avar(self, rig, ref):
        """
        Factory method to create a standard avar for this group.
        """
        influences = [ref]
        '''
        if self.surface:
            influences.append(self.surface)
        '''
        avar = self._CLS_AVAR(influences)
        return avar

    def configure_avar(self, rig, avar):
        """
        This method is called as soon as we access or create an avar.
        Use it to configure the avar automatically.
        """
        if avar.surface is None and self.surface:
            avar.surface = self.surface

    def build_abstract_avar(self, rig, cls_ctrl, avar, **kwargs):
        """
        Factory method that create an avar that is not affiliated with any influence and is only used for connections.
        :param rig: The parent rig.
        :param cls_ctrl: The class definition to use for the ctrl.
        :param avar: The avar class instance to use.
        :param kwargs: Any additional keyword arguments will be sent to the avar build method.
        :return:
        """
        avar._CLS_CTRL = cls_ctrl  # Hack, find a more elephant way.
        avar.build(
            rig,
            grp_rig=self.grp_rig,
            callibrate_doritos=False,  # We'll callibrate ourself since we're connecting manually.
            constraint=False,  # We are only using the avar to control
            **kwargs
        )
        if avar.grp_anm:
            avar.grp_anm.setParent(self.grp_anm)
        if avar.grp_rig:
            avar.grp_rig.setParent(self.grp_rig)

        return avar

    #
    # AvarGrps can be decomposed in quadrants.
    # This allow us generically support modules that have a left/right/upp/low side. (ex: eyelids, lips, etc)
    #

    def build_and_connect_abstract_avar(self, rig, avar, children_avars, cls_ctrl, connect_ud=True, connect_lr=True, connect_fb=True, calibrate=True):
        self.build_abstract_avar(rig, cls_ctrl, avar)

        for child_avar in children_avars:
            if connect_ud:
                libRigging.connectAttr_withLinearDrivenKeys(avar.attr_ud, child_avar.attr_ud)
            if connect_lr:
                libRigging.connectAttr_withLinearDrivenKeys(avar.attr_lr, child_avar.attr_lr)
            if connect_fb:
                libRigging.connectAttr_withLinearDrivenKeys(avar.attr_fb, child_avar.attr_fb)

        if calibrate:
            avar.calibrate()

    def create_abstract_avar_center(self, rig, cls_ctrl, ref=None):
        """
        A center abstract Avar is used to control ALL the avars.
        ex: Controlling the whole eye or mouth section.
        """
        if ref is None:
            ref = self.parent
        if ref is None:
            raise Exception("Can't build abstract avar for the global section. No reference influence found!")

        name = '{0}_All'.format(self.get_module_name())
        avar = self.create_abstract_avar(rig, cls_ctrl, ref, cls_avar=self._CLS_AVAR_ALL)
        avar.name = name

        return avar

    def create_abstract_avar_left_side(self, rig, cls_ctrl, ref=None):
        if ref is None:
            ref = self.jnt_l_mid
        if ref is None:
            raise Exception("Can't build abstract avar for the left section. No reference influence found!")

        name = 'L_{0}'.format(self.get_module_name())
        avar = self.create_abstract_avar(rig, cls_ctrl, ref, cls_avar=self._CLS_AVAR)
        avar.name = name

        return avar

    def create_abstract_avar_right_side(self, rig, avar, cls_ctrl, ref=None):
        if ref is None:
            ref = self.jnt_r_mid
        if ref is None:
            raise Exception("Can't build abstract avar for the left section. No reference influence found!")

        # Create l ctrl
        name = 'R_{0}'.format(self.get_module_name())
        avar = self.create_abstract_avar(rig, cls_ctrl, ref, cls_avar=self._CLS_AVAR)
        avar.name = name

        avar.calibrate()

        return avar

    def create_abstract_avar_upp_side(self, rig, avar, cls_ctrl, ref=None):
        if ref is None:
            ref = self.jnt_upp_mid
        if ref is None:
            raise Exception("Can't build abstract avar for the upper section. No reference influence found!")

        # Resolve avar name
        avar_upp_basename = '{0}Upp'.format(self.get_module_name())
        nomenclature_upp = rig.nomenclature(ref.name())
        nomenclature_upp.tokens = [avar_upp_basename]
        avar_upp_name = nomenclature_upp.resolve()

        avar = self.create_abstract_avar(rig, cls_ctrl, ref)
        avar.name = avar_upp_name

        return avar

    def create_abstract_avar_low_side(self, rig, avar, cls_ctrl, ref=None):
        if ref is None:
            ref = self.jnt_low_mid
        if ref is None:
            raise Exception("Can't build abstract avar for the lower section. No reference influence found!")

        # Resolve avar name
        avar_low_basename = '{0}Low'.format(self.get_module_name())
        nomenclature_low = rig.nomenclature(ref.name())
        nomenclature_low.tokens = [avar_low_basename]
        avar_low_name = nomenclature_low.resolve()

        avar = self.create_abstract_avar(rig, cls_ctrl, ref)
        avar.name = avar_low_name

        return avar

class AvarGrpOnSurface(AvarGrp):
    _CLS_AVAR = rigFaceAvar.AvarFollicle
    _CLS_AVAR_ALL = rigFaceAvar.AvarFollicleGlobal

    def __init__(self, *args, **kwargs):
        super(AvarGrpOnSurface, self).__init__(*args, **kwargs)
        self.surface = None

    '''
    @libPython.cached_property()
    def surface(self):
        fn_is_nurbsSurface = lambda obj: libPymel.isinstance_of_shape(obj, pymel.nodetypes.NurbsSurface)
        objs = filter(fn_is_nurbsSurface, self.input)
        return next(iter(objs), None)
    '''

    @classModule.decorator_uiexpose
    def create_surface(self, *args, **kwargs):
        """
        Expose the function in the ui, using the decorator.
        """
        return super(AvarGrpOnSurface, self).create_surface(*args, **kwargs)

class AvarGrpAim(AvarGrp):
    _CLS_AVAR = rigFaceAvar.AvarAim
    SHOW_IN_UI = False

class AvarGrpAreaOnSurface(AvarGrpOnSurface):
    """
    This module will build AvarGrps with extra abstract avars.
    """
    _CLS_CTRL_LFT = CtrlFaceHorizontal
    _CLS_CTRL_RGT = CtrlFaceHorizontal  # the negative scale of it's parent will flip it's shape
    _CLS_CTRL_UPP = CtrlFaceUpp
    _CLS_CTRL_LOW = CtrlFaceLow
    _CLS_CTRL_ALL = CtrlFaceAll
    SHOW_IN_UI = False

    CREATE_MACRO_AVAR_HORIZONTAL = True
    CREATE_MACRO_AVAR_VERTICAL = True
    CREATE_MACRO_AVAR_ALL = True

    def __init__(self, *args, **kwargs):
        super(AvarGrpAreaOnSurface, self).__init__(*args, **kwargs)
        self.avar_all = None
        self.avar_l = None
        self.avar_r = None
        self.avar_upp = None
        self.avar_low = None

    def add_avars(self, attr_holder):
        pass

    def connect_global_avars(self):
        pass

    def get_multiplier_u(self):
        """
        Since we are using the same plane for the eyebrows, we want to attenget_multiplier_lruate the relation between the LR avar
        and the plane V coordinates.
        In the best case scenario, at LR -1, the V coordinates of the BrowInn are 0.5 both.
        """
        base_u, base_v = self.get_base_uv()
        return abs(base_u - 0.5) * 2.0

    def build_and_connect_abstract_avar_all(self, rig, avar_parent, avar_children, cls_ctrl, connect_ud=True, connect_lr=True, connect_fb=True, calibrate=True):
        self.handle_surface(rig)  # ensure we have a surface
        pos = libRigging.get_point_on_surface_from_uv(self.surface, 0.5, 0.5)
        jnt_tm = pymel.datatypes.Matrix(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            pos.x, pos.y, pos.z, 1
        )

        self.build_abstract_avar(rig, cls_ctrl, avar_parent, jnt_tm=jnt_tm, ctrl_tm=jnt_tm, obj_mesh=self.surface)

        for avar_child in avar_children:
            if connect_ud:
                libRigging.connectAttr_withLinearDrivenKeys(avar_parent.attr_ud, avar_child.attr_ud)
            if connect_lr:
                libRigging.connectAttr_withLinearDrivenKeys(avar_parent.attr_lr, avar_child.attr_lr)
            if connect_fb:
                libRigging.connectAttr_withLinearDrivenKeys(avar_parent.attr_fb, avar_child.attr_fb)

        if calibrate:
            avar_parent.calibrate()

    def build_and_connect_abstract_avar_horizontal(self, rig, avar_parent, avar_middle, avar_children, cls_ctrl, connect_ud=True, connect_lr=True, connect_fb=True, calibrate=True):
        self.build_abstract_avar(rig, cls_ctrl, avar_parent)

        pos_s = avar_middle.jnt.getTranslation(space='world')
        pos_e = avar_parent.jnt.getTranslation(space='world')

        for avar_child in avar_children:
            # We don't want to connect the middle Avar.
            if avar_child == avar_middle:
                continue

            pos = avar_child.jnt.getTranslation(space='world')

            # Resolve the delta using the x axis.
            # We ensure that the delta is always between 0 and 1.
            delta = (pos.x - pos_s.x) / (pos_e.x - pos_s.x)
            delta = max(0, delta)
            delta = min(delta, 1)

            if connect_ud:
                libRigging.connectAttr_withLinearDrivenKeys(avar_parent.attr_ud, avar_child.attr_ud,  kv=(-delta,0.0,delta))
            if connect_lr:
                libRigging.connectAttr_withLinearDrivenKeys(avar_parent.attr_lr, avar_child.attr_lr,  kv=(-delta,0.0,delta))
            if connect_fb:
                libRigging.connectAttr_withLinearDrivenKeys(avar_parent.attr_fb, avar_child.attr_fb,  kv=(-delta,0.0,delta))

        if calibrate:
            avar_parent.calibrate()

    def build_and_connect_abstract_avar_vertical(self, rig, avar_parent, avar_middle, avar_children, cls_ctrl, **kwargs):
        self.build_and_connect_abstract_avar(
            rig,
            avar_parent,
            avar_children,
            cls_ctrl,
            connect_ud=False, connect_lr=False, connect_fb=True,
            **kwargs
        )

    def build(self, rig, **kwargs):
        super(AvarGrpAreaOnSurface, self).build(rig, **kwargs)

        # Create left avar if necessary
        ref = self.jnt_l_mid
        if self.CREATE_MACRO_AVAR_HORIZONTAL and ref :
            if not self.avar_l:
                self.avar_l = self.create_abstract_avar_left_side(rig, self._CLS_CTRL_LFT, ref)
            self.build_and_connect_abstract_avar_horizontal(rig, self.avar_l, self.avar_mid, self.avars_l, self._CLS_CTRL_LFT)

        # Create right avar if necessary
        ref = self.jnt_r_mid
        if self.CREATE_MACRO_AVAR_HORIZONTAL and ref:
            # Create l ctrl
            if not self.avar_r:
                self.avar_r = self.create_abstract_avar_right_side(rig, self._CLS_CTRL_RGT, ref)
            self.build_and_connect_abstract_avar_horizontal(rig, self.avar_r, self.avar_mid, self.avars_r, self._CLS_CTRL_RGT)

        # Create upp avar if necessary
        ref = self.jnt_upp_mid
        if self.CREATE_MACRO_AVAR_VERTICAL and ref:
            if self.avar_upp is None:
                self.avar_upp = self.create_abstract_avar_upp_side(rig, self._CLS_CTRL_UPP, ref)
            self.build_and_connect_abstract_avar_vertical(rig, self.avar_upp, self.avar_mid, self.avars_upp, self._CLS_CTRL_UPP)

        # Create low avar if necessary
        ref = self.jnt_low_mid
        if self.CREATE_MACRO_AVAR_VERTICAL and ref:
            if self.avar_low is None:
                self.avar_low = self.create_abstract_avar_low_side(rig, self._CLS_CTRL_LOW, ref)
            self.build_and_connect_abstract_avar_vertical(rig, self.avar_low, self.avar_mid, self.avars_low, self._CLS_CTRL_LOW)

        # Create all avar if necessary
        ref = self.parent
        if self.CREATE_MACRO_AVAR_ALL and ref:
            if not self.avar_all:
                self.avar_all = self.create_abstract_avar_center(rig, self._CLS_CTRL_ALL, ref)
            self.build_and_connect_abstract_avar_all(rig, self.avar_all, self.avars, self._CLS_CTRL_ALL)

    def unbuild(self):
        if self.avar_l:
            self.avar_l.unbuild()
        if self.avar_r:
            self.avar_r.unbuild()
        if self.avar_upp:
            self.avar_upp.unbuild()
        if self.avar_low:
            self.avar_low.unbuild()
        super(AvarGrpAreaOnSurface, self).unbuild()
