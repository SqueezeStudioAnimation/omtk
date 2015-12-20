import pymel.core as pymel
from classCtrl import BaseCtrl
from classModule import Module
from classNode import Node
from omtk.libs import libRigging, libAttr, libFormula, libPymel
from className import Name
import functools


class CtrlIk(BaseCtrl):
    kAttrName_State = 'ikFk'

    def __init__(self, *args, **kwargs):
        super(CtrlIk, self).__init__(*args, **kwargs)
        self.m_attState = None

    def build(self, *args, **kwargs):
        super(CtrlIk, self).build(*args, **kwargs)
        assert (self.node is not None)
        pymel.addAttr(self.node, longName=self.kAttrName_State)
        self.m_attState = getattr(self.node, self.kAttrName_State)
        return self.node

    def unbuild(self, *args, **kwargs):
        super(CtrlIk, self).unbuild(*args, **kwargs)
        self.m_attState = None


class CtrlIkSwivel(BaseCtrl):
    def build(self, _oLineTarget=False, *args, **kwargs):
        super(CtrlIkSwivel, self).build(*args, **kwargs)
        assert (self.node is not None)
        make = self.node.getShape().create.inputs()[0]
        make.radius.set(make.radius.get() * 0.5)
        make.degree.set(1)
        make.sections.set(4)

        # Create line
        if _oLineTarget is True:
            ctrl_shape = self.node.getShape()
            line_shape = pymel.createNode('annotationShape')
            line_transform = line_shape.getParent()
            pymel.connectAttr(ctrl_shape.worldMatrix, line_shape.dagObjectMatrix[0], force=True)
            line_transform.setParent(self.offset)
            pymel.pointConstraint(_oLineTarget, line_transform)

        return self.node


class SoftIkNode(Node):
    def __createNode__(self, *args, **kwargs):
        return pymel.createNode('network')

    def build(self):
        super(SoftIkNode, self).build()
        formula = libFormula.Formula()
        fn_add_attr = functools.partial(libAttr.addAttr, self.node, hasMinValue=True, hasMaxValue=True)
        formula.inMatrixS = fn_add_attr(longName='inMatrixS', dt='matrix')
        formula.inMatrixE = fn_add_attr(longName='inMatrixE', dt='matrix')
        formula.inRatio = fn_add_attr(longName='inRatio', at='float')
        formula.inStretch = fn_add_attr(longName='inStretch', at='float')
        formula.inChainLength = fn_add_attr(longName='inChainLength', at='float', defaultValue=1.0)

        # inDistance is the distance between the start of the chain and the ikCtrl
        formula.inDistance = "inMatrixS~inMatrixE"
        # distanceSoft is the distance before distanceMax where the softIK kick in.
        # ex: For a chain of length 10.0 with a ratio of 0.1, the distanceSoft will be 1.0.
        formula.distanceSoft = "inChainLength*inRatio"
        # distanceSafe is the distance where there's no softIK.
        # ex: For a chain of length 10.0 with a ratio of 0.1, the distanceSafe will be 9.0.
        formula.distanceSafe = "inChainLength-distanceSoft"
        # This represent the soft-ik state
        # When the soft-ik kick in, the value is 0.0.
        # When the stretch kick in, the value is 1.0.
        # |-----------|-----------|----------|
        # -1          0.0         1.0         +++
        # -dBase      dSafe       dMax
        formula.deltaSafeSoft = "(inDistance-distanceSafe)/distanceSoft"
        # Hack: Prevent potential division by zero when soft-ik is desactivated
        formula.deltaSafeSoft = libRigging.create_utility_node('condition',
                                                               firstTerm=formula.distanceSoft,
                                                               secondTerm=0.0,
                                                               colorIfTrueR=0.0,
                                                               colorIfFalseR=formula.deltaSafeSoft
                                                               ).outColorR

        # outDistanceSoft is the desired ikEffector distance from the chain start after aplying the soft-ik
        # If there's no stretch, this will be directly applied to the ikEffector.
        # If there's stretch, this will be used to compute the amount of stretch needed to reach the ikCtrl
        # while preserving the shape.
        formula.outDistanceSoft = "(distanceSoft*(1-(e^(deltaSafeSoft*-1))))+distanceSafe"

        # Affect ikEffector distance only where inDistance if bigger than distanceSafe.
        formula.outDistance = libRigging.create_utility_node('condition',
                                                             operation=2,
                                                             firstTerm=formula.deltaSafeSoft,
                                                             secondTerm=0.0,
                                                             colorIfTrueR=formula.outDistanceSoft,
                                                             colorIfFalseR=formula.inDistance
                                                             ).outColorR
        # Affect ikEffector when we're not using stretching
        formula.outDistance = libRigging.create_utility_node('blendTwoAttr',
                                                             input=[formula.outDistance, formula.inDistance],
                                                             attributesBlender=formula.inStretch).output

        #
        # Handle Stretching
        #

        # If we're using softIk AND stretchIk, we'll use the outRatioSoft to stretch the joints enough so
        # that the ikEffector reach the ikCtrl.
        formula.outStretch = "inDistance/outDistanceSoft"

        # Apply the softIK only AFTER the distanceSafe
        formula.outStretch = libRigging.create_utility_node('condition',
                                                            operation=2,
                                                            firstTerm=formula.inDistance,
                                                            secondTerm=formula.distanceSafe,
                                                            colorIfTrueR=formula.outStretch,
                                                            colorIfFalseR=1.0
                                                            ).outColorR

        # Apply stretching only if inStretch is ON
        formula.outStretch = libRigging.create_utility_node('blendTwoAttr',
                                                            input=[1.0, formula.outStretch],
                                                            attributesBlender=formula.inStretch).output

        #
        # Connect outRatio and outStretch to our softIkNode
        #
        # fnAddAttr(longName='outTranslation', dt='float3')
        formula.outRatio = "outDistance/inDistance"
        attr_ratio = fn_add_attr(longName='outRatio', at='float')
        pymel.connectAttr(formula.outRatio, attr_ratio)

        attr_stretch = fn_add_attr(longName='outStretch', at='float')
        pymel.connectAttr(formula.outStretch, attr_stretch)


# Todo: Support more complex IK limbs (ex: 2 knees)
class IK(Module):
    def __init__(self, *args, **kwargs):
        super(IK, self).__init__(*args, **kwargs)
        self.iCtrlIndex = 2
        self.ctrlIK = None
        self.ctrl_swivel = None
        self.chain_length = None

    def calc_swivel_pos(self):
        pos_start = self.input[0].getTranslation(space='world')
        pos_end = self.input[self.iCtrlIndex].getTranslation(space='world')
        ratio = self.input[1].t.get().length() / self.chain_length
        pos_swivel_base = (pos_end - pos_start) * ratio + pos_start
        dir_swivel = (self.input[1].getTranslation(space='world') - pos_swivel_base).normal()
        return pos_swivel_base + dir_swivel * self.chain_length

    def build(self, orient_ik_ctrl=True, constraint=False, *args, **kwargs):
        super(IK, self).build(*args, **kwargs)

        # Duplicate input chain (we don't want to move the hierarchy)
        # Todo: implement a duplicate method in omtk.libs.libPymel.PyNodeChain
        # Create ikChain and fkChain
        self._chain_ik = pymel.duplicate(self.input, renameChildren=True, parentOnly=True)
        for oInput, oIk, in zip(self.input, self._chain_ik):
            oIk.rename(self._name_rig.resolve('ik'))
        self._chain_ik[0].setParent(self._oParent)  # Trick the IK system (temporary solution)

        obj_s = self._chain_ik[0]
        obj_e = self._chain_ik[self.iCtrlIndex]

        # Compute chain length
        self.chain_length = self._chain.length()

        # Compute swivel position
        p3SwivelPos = self.calc_swivel_pos()

        # Create ikChain
        ikChainGrp_name = self._name_rig.resolve('ikChain')
        ikChainGrp = pymel.createNode('transform', name=ikChainGrp_name, parent=self.grp_rig)
        ikChainGrp.setMatrix(obj_s.getMatrix(worldSpace=True), worldSpace=True)
        obj_s.setParent(ikChainGrp)

        # Create ikEffector
        _ik_solver_name = self._name_rig.resolve('ikHandle')
        _ik_effector_name = self._name_rig.resolve('ikEffector')
        self._ik_handle, _ik_effector = pymel.ikHandle(startJoint=obj_s, endEffector=obj_e, solver='ikRPsolver')
        self._ik_handle.rename(_ik_solver_name)
        self._ik_handle.setParent(ikChainGrp)
        _ik_effector.rename(_ik_effector_name)

        # Create ctrls
        if not isinstance(self.ctrlIK, CtrlIk): self.ctrlIK = CtrlIk()
        self.ctrlIK.build()
        # self.ctrlIK = CtrlIk(_create=True)
        self.ctrlIK.setParent(self.grp_anm)
        ctrlIK_name = self._name_anm.resolve('ik')
        self.ctrlIK.rename(ctrlIK_name)
        self.ctrlIK.offset.setTranslation(obj_e.getTranslation(space='world'), space='world')
        if orient_ik_ctrl is True:
            self.ctrlIK.offset.setRotation(obj_e.getRotation(space='world'), space='world')

        if not isinstance(self.ctrl_swivel, CtrlIkSwivel): self.ctrl_swivel = CtrlIkSwivel()
        self.ctrl_swivel.build()
        # self.ctrl_swivel = CtrlIkSwivel(_oLineTarget=self.input[1], _create=True)
        self.ctrl_swivel.setParent(self.grp_anm)
        self.ctrl_swivel.rename(self._name_anm.resolve('ikSwivel'))
        self.ctrl_swivel.offset.setTranslation(p3SwivelPos, space='world')
        self.ctrl_swivel.offset.setRotation(self.input[self.iCtrlIndex - 1].getRotation(space='world'), space='world')
        self.swivelDistance = self.chain_length  # Used in ik/fk switch

        #
        # Create softIk node and connect user accessible attributes to it.
        #
        oAttHolder = self.ctrlIK
        fnAddAttr = functools.partial(libAttr.addAttr, hasMinValue=True, hasMaxValue=True)
        attInRatio = fnAddAttr(oAttHolder, longName='SoftIkRatio', niceName='SoftIK', defaultValue=0, minValue=0,
                               maxValue=.5, k=True)
        attInStretch = fnAddAttr(oAttHolder, longName='Stretch', niceName='Stretch', defaultValue=0, minValue=0,
                                 maxValue=1.0, k=True)

        # Create and configure SoftIK solver
        rig_softIkNetwork = SoftIkNode()
        rig_softIkNetwork.build()
        pymel.connectAttr(attInRatio, rig_softIkNetwork.inRatio)
        pymel.connectAttr(attInStretch, rig_softIkNetwork.inStretch)
        pymel.connectAttr(ikChainGrp.worldMatrix, rig_softIkNetwork.inMatrixS)
        pymel.connectAttr(self.ctrlIK.worldMatrix, rig_softIkNetwork.inMatrixE)
        attr_distance = libFormula.parse('distance*globalScale',
                                         distance=self.chain_length,
                                         globalScale=self.grp_rig.globalScale)
        pymel.connectAttr(attr_distance, rig_softIkNetwork.inChainLength)

        # Constraint effector
        attOutRatio = rig_softIkNetwork.outRatio
        attOutRatioInv = libRigging.create_utility_node('reverse', inputX=rig_softIkNetwork.outRatio).outputX
        pymel.select(clear=True)
        pymel.select(self.ctrlIK, ikChainGrp, self._ik_handle)
        pointConstraint = pymel.pointConstraint()
        pointConstraint.rename(pointConstraint.name().replace('pointConstraint', 'softIkConstraint'))
        pymel.select(pointConstraint)
        weight_inn, weight_out = pointConstraint.getWeightAliasList()
        pymel.connectAttr(attOutRatio, weight_inn)
        pymel.connectAttr(attOutRatioInv, weight_out)

        # Constraint joints stretch
        attOutStretch = rig_softIkNetwork.outStretch
        attr_joint_stretch = libRigging.create_utility_node('multiplyDivide',
                                                            input1X=rig_softIkNetwork.outStretch,
                                                            input2X=self.grp_rig.globalScale).outputX
        num_jnts = len(self._chain_ik)
        for i in range(1, num_jnts):
            obj = self._chain_ik[i]
            pymel.connectAttr(
                libRigging.create_utility_node('multiplyDivide',
                                               input1X=attr_joint_stretch,
                                               input1Y=attr_joint_stretch,
                                               input1Z=attr_joint_stretch,
                                               input2=obj.t.get()).output,
                obj.t, force=True)

        # Connect rig -> anm
        pymel.orientConstraint(self.ctrlIK, obj_e, maintainOffset=True)
        pymel.poleVectorConstraint(self.ctrl_swivel, self._ik_handle)

        # Connect to parent
        if libPymel.is_valid_PyNode(self.parent):
            pymel.parentConstraint(self.parent, ikChainGrp, maintainOffset=True)

        if constraint:
            for source, target in zip(self._chain_ik, self._chain):
                pymel.parentConstraint(source, target)
