import pymel.core as pymel
import logging
from classNameMap import NameMap
from classRigElement import RigElement

'''
This is the baseclass for anything that can be Build/Unbuild


A RigPart serialized respect the maya architecture and can be vulgarised to a node.
This allow us to expend the autorigger to software supporting compounds (xsi) or digital assets (houdini).

To build a RigPart, use the build and unbuild function.
To manage a rigpart, use the build() and unbuild() function.
Also, a rigpart have usefull properties like @inputs and @outputs.

'''
class RigPart(RigElement):

    # RigElement overrides
    def isBuilt(self):
        return self.grp_anm is not None or self.grp_rig is not None

    # Allow self.PostInputSet() to be called automaticly when self.inputs is set.
    @property
    def inputs(self):
        return self.__dict__['_inputs']

    @inputs.setter
    def inputs(self, val):
        self.__dict__['_inputs'] = val
        self._post_setattr_inputs()

    @property
    def outputs(self):
        return self.__dict__['_outputs']


    def __init__(self, _inputs=[], *args, **kwargs):
        super(RigPart, self).__init__(*args, **kwargs)
        self.iCtrlIndex = 2
        self.grp_anm = None
        self.grp_rig = None
        self._pNameMapAnm = None
        self._pNameMapRig = None

        self.inputs = _inputs # since we're using hook on inputs, assign it last!

    def __repr__(self):
        # TODO: Never crash on __repr__
        assert(hasattr(self, '_pNameMapAnm'))
        return '{0} ({1})'.format(str(self._pNameMapAnm), self.__class__.__name__ )

    # Even when nothing is build, it's usefull to access properties like namemaps.
    # This method is called automaticly when self.inputs is changed.
    def _post_setattr_inputs(self):
        oRef = next(iter(self.inputs), None)
        if oRef is not None:
            self._pNameMapAnm = NameMap(oRef, _sType='anm')
            self._pNameMapRig = NameMap(oRef, _sType='rig')
            self._oParent = oRef.getParent() if oRef is not None else None

    def build(self, _bCreateGrpAnm=True, _bCreateGrpRig=True, *args, **kwargs):
        if len(self.inputs) == 0:
            raise Exception("No inputs defined for {0}".format(self))
        assert(hasattr(self, '_pNameMapAnm'))
        assert(self._pNameMapAnm is not None)
        assert(hasattr(self, '_pNameMapRig'))
        assert(self._pNameMapRig is not None)

        logging.info('Building {0}'.format(self._pNameMapRig.Serialize()))

        if len(self.inputs) == 0:
            logging.error("[RigPart:Build] Can't build, inputs is empty"); return False

        if _bCreateGrpAnm:
            self.grp_anm = pymel.createNode('transform', name=self._pNameMapAnm.Serialize(self.__class__.__name__.lower(), _sType='anms'))
        if _bCreateGrpRig:
            self.grp_rig = pymel.createNode('transform', name=self._pNameMapRig.Serialize(self.__class__.__name__.lower(), _sType='rigs'))

    def unbuild(self):
        if self.grp_anm is not None:
            pymel.delete(self.grp_anm)
            self.grp_anm = None
        if self.grp_rig is not None:
            pymel.delete(self.grp_rig)
            self.grp_rig = None

    # Used in libSerialization
    def __getNetworkName__(self):
        assert(hasattr(self, '_pNameMapRig'))
        assert(self._pNameMapRig)
        return self._pNameMapRig.Serialize(self.__class__.__name__, _sType='net')

    # Overwritten from Serializable
    def __createMayaNetwork__(self):
        return pymel.createNode('network', name=self._pNameMapAnm.Serialize(_sType='net'))