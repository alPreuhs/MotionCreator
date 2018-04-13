import vtk
import numpy as np


##Class that can be used as InteractionStyle, where events can be catched
class InteractorStyle(vtk.vtkInteractorStyleTrackballCamera):

    def __init__(self, motion_creator_instance, parent=None):
        self.mocrin = motion_creator_instance
        self.initialModelViewMatrix = vtk.vtkMatrix4x4()

        if (parent is not None):
            self.parent = parent
        else:
            self.parent = vtk.vtkRenderWindowInteractor()

        self.start_position = self.parent.GetRenderWindow().GetRenderers().GetFirstRenderer().GetActiveCamera().GetPosition()
        self.movement = False
        self.rotation = False
        # self.AddObserver("KeyPressEvent", self.keyPress)
        # self.AddObserver("QMouseEvent", self.mouseEvent)
        # self.AddObserver("MouseEvent", self.mouseEvent)
        # self.AddObserver("MouseWheelForwardEvent", self.mouseEvent)
        # self.AddObserver("MouseWheelBackwardEvent", self.mouseEvent)
        # self.AddObserver("RightButtonPressEvent", self.mouseEvent)
        self.AddObserver("LeftButtonReleaseEvent", self.LeftButtonReleaseEvent)
        self.AddObserver("MiddleButtonReleaseEvent", self.MiddleButtonReleaseEvent)
        self.AddObserver("RightButtonReleaseEvent", self.RigthButtonReleaseEvent)

        self.GetCameraParameters
        #
        # self.AddObserver("WheelEvent", self.mouseEvent)
        # self.AddObserver("wheelEvent", self.mouseEvent)

    def RigthButtonReleaseEvent(self, event, m):
        print("")

    #    def onCustomInitialization(self):
    #        renderers = self.parent.GetRenderWindow().GetRenderers()
    #        cam = renderers.GetFirstRenderer().GetActiveCamera()
    #        mat = cam.GetModelViewTransformMatrix()
    #        self.initialModelViewMatrix.DeepCopy(mat)
    #        print()
    #        print("initialzierd Model matrix:")
    #        for i in range(4):
    #            for j in range(4):
    #                print(mat.GetElement(i,j), end=' ')
    #            print()
    #        print()
    #        print()
    #
    def MiddleButtonReleaseEvent(self, event, m):
        # print("Middle Button was Pressed")
        self.movement = True
        self.OnMiddleButtonUp()
        self.GetCameraParameters()
        self.movement = False
        return

    def LeftButtonReleaseEvent(self, event, m):
        # print("Left Button was Pressed")
        self.rotation = True
        self.OnLeftButtonUp()
        self.GetCameraParameters()
        self.rotation = False
        return

    def GetCameraParameters(self):

        renderers = self.parent.GetRenderWindow().GetRenderers()
        cam = renderers.GetFirstRenderer().GetActiveCamera()
        roll = cam.GetRoll()
        model = cam.GetModelViewTransformObject()
        modelmatrix = model.GetMatrix()

        if (self.rotation == True):
            orientation = cam.GetOrientation()
            r_sag = orientation[0]
            r_ax = orientation[1]
            r_cor = orientation[2]
            self.start_position = cam.GetPosition()
            self.motion_parameters = np.array([0, 0, 0, r_ax, r_cor, r_sag])
            self.mocrin.vtk_motion_to_graphics_view(self.motion_parameters)
            print(
                "orientation: rot_sag:" + str(r_sag) + "°  rot_cor:" + str(r_cor) + "° + rot_ax: " + str(r_ax) + "°")

        if (self.movement == True):
            current_position = cam.GetPosition()
            t_cor = -(current_position[0] - self.start_position[0])
            t_sag = -(current_position[1] - self.start_position[1])
            t_ax = -(current_position[2] - self.start_position[2])
            self.motion_parameters = np.array([t_ax, t_cor, t_sag, 0, 0, 0])
            self.mocrin.vtk_motion_to_graphics_view(self.motion_parameters)
            # print(
            #    "shifting of position: t_cor:" + str(t_cor) + "; t_sag:" + str(t_sag) + "; t_ax:" + str(t_ax) + "\n")

        # print()
        # for i in range(4):
        #     # print("row {}".format(i))
        #     print()
        #     for j in range(4):
        #         # print("{} {}".format(i, j))
        #         print(modelmatrix.GetElement(i, j), end=' ')

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
