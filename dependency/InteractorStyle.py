import vtk


##Class that can be used as InteractionStyle, where events can be catched
class InteractorStyle(vtk.vtkInteractorStyleTrackballCamera):

    def __init__(self, parent=None):
        self.initialModelViewMatrix = vtk.vtkMatrix4x4()

        if (parent is not None):
            self.parent = parent
        else:
            self.parent = vtk.vtkRenderWindowInteractor()





    def keyPress(self, obj, event):
        print("huhu")
        key = self.parent.GetKeySym()
        if key == 'space':
            renderers = self.parent.GetRenderWindow().GetRenderers()
            cam = renderers.GetFirstRenderer().GetActiveCamera()
            # cam.Zoom(4.0)
            roll = cam.GetRoll()
            size = self.parent.GetSize()
            cam.GetFocalPoint()
            cam.UpdateViewport(renderers.GetFirstRenderer())
            cam.GetOrientation()
            cam.GetPosition()
