from . import modelInteractiveCtrl


class ModelNonInteractiveCtrl(modelInteractiveCtrl.ModelInteractiveCtrl):
    """
    Same system as the ModelInteractiveCtrl.
    However instead of using a follicle to position the controller at the correct location on the mesh,
    we use simple constraint. This is necessary to optimize performance on dense meshes.
    """
    _BIND_METHOD = modelInteractiveCtrl.ModelInteractiveCtrlBindMethod.Constraint


def register_plugin():
    return ModelNonInteractiveCtrl
