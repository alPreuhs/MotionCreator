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

        self.reference_position = self.parent.GetRenderWindow().GetRenderers().GetFirstRenderer().GetActiveCamera().GetPosition()

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

    def apply_rotation(self, cam):
        orientation = cam.GetOrientation()
        r_sag = orientation[0]
        r_ax = orientation[1]
        r_cor = orientation[2]
        self.motion_parameters = np.array([0, 0, 0, r_ax, r_cor, r_sag])
        self.mocrin.vtk_motion_to_graphics_view(self.motion_parameters, True, False)
        # print("orientation: rot_sag:" + str(r_sag) + "°  rot_cor:" + str(r_cor) + "° rot_ax: " + str(r_ax) + "°")
        # print("Current position: " + str(cam.GetPosition()))

    def apply_shifting(self, current_position):
        t_cor = -(current_position[0] - self.reference_position[0])
        t_sag = -(current_position[1] - self.reference_position[1])
        t_ax = (current_position[2] - self.reference_position[2])
        self.motion_parameters = np.array([t_ax, t_cor, t_sag, 0, 0, 0])
        self.mocrin.vtk_motion_to_graphics_view(self.motion_parameters, False, True)

    def GetCameraParameters(self):
        renderers = self.parent.GetRenderWindow().GetRenderers()
        camera = renderers.GetFirstRenderer().GetActiveCamera()
        roll = camera.GetRoll()
        model = camera.GetModelViewTransformObject()
        modelmatrix = model.GetMatrix()

        if (self.rotation == True):
            self.apply_rotation(camera)
        current_position = camera.GetPosition()
        if (self.movement == True):
            self.apply_shifting(current_position)

        # print("Interactor Style:")
        # print("Distance: " + str(camera.GetDistance()))
        # print("focal: " + str(camera.GetFocalPoint()))
        # print("view Up: " + str(cam.GetViewUp()))
        # print("Position: " + str(camera.GetPosition()))
        # print("Orientation: " + str(camera.GetOrientation()) + "\n")

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
