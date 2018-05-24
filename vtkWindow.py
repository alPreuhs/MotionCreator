import numpy as np
import vtk
from math import *
from PyQt5 import QtWidgets
from dependency.InteractorStyle import InteractorStyle
from dependency.RenderWindowInteractor import *
from include import help_functions


##TODO synchronize shifting "translation" - rotation working

## class that needs a qtFrame and places a vtk renderwindow inside
class vtkWindow():
    def vtkWidget(self, qtFrame, motion_creator_instance):
        # the center computation might seem to be a bit complicated however what we do is:
        # the center_of_rotation gives the center of rotation in pixel coordinates
        self.vl = QtWidgets.QGridLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(qtFrame)
        self.vl.addWidget(self.vtkWidget)
        self.vl.setContentsMargins(0, 0, 0, 0)
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        # Create an actor
        self.arrowSource = vtk.vtkArrowSource()
        reader = vtk.vtkSTLReader()
        reader.SetFileName('include/Head_Phantom.stl')
        reader.Update()

        polydata = reader.GetOutput()
        pd_center = polydata.GetCenter()
        pd_bounds = polydata.GetBounds()
        #   spacing = polydata.GetSpacing()
        # todo: use bounds to scale the translation stuff
        transform = vtk.vtkTransform()

        R = help_functions.get_rotation(-90, 0,
                                        0)  # (nicken (LinksUnten, von schulter zu schuler links oben, verneinen rechts Oben)
        t = np.matrix([[1.0, 0, 0, -pd_center[0]],
                       [0, 1.0, 0, -pd_center[1]],
                       [0, 0, 1.0, -pd_center[2] + 206 * (1 / 3)],  # 1054.8
                       [0, 0, 0, 1]])
        transform.Identity()
        matrix = help_functions.GetVTKMatrix(R * t)
        transform.Concatenate(matrix)
        # polydata.Update();
        mapper2 = vtk.vtkPolyDataMapper()
        mapper2.SetInputConnection(reader.GetOutputPort())

        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetInputData(polydata)
        transformFilter.SetTransform(transform)
        transformFilter.Update()
        pdm = vtk.vtkPolyDataMapper()
        pdm.SetInputConnection(transformFilter.GetOutputPort())
        # Create a mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.arrowSource.GetOutputPort())
        #  mapper.SetScalarRange(scalar_range)
        self.actor = vtk.vtkActor()
        # self.actor.GetProperty().SetColor(color[0], color[1], color[2])
        # self.actor.SetMapper(mapper)
        self.actor.SetMapper(pdm)
        self.ren.SetBackground(.1, .2, .3)
        self.ren.AddActor(self.actor)
        self.vtkWidget.Initialize()
        self.iren.Initialize()
        self.interactorStyle = InteractorStyle(motion_creator_instance, parent=self.iren)
        self.iren.SetInteractorStyle(self.interactorStyle)

        qtFrame.setLayout(self.vl)

        self.initial_camera = vtk.vtkCamera()
        self.initial_camera.DeepCopy(self.ren.GetActiveCamera())

    def init_camera(self):
        initial_camera = vtk.vtkCamera()
        initial_camera.DeepCopy(self.initial_camera)
        self.ren.SetActiveCamera(initial_camera)
        self.iren.Initialize()

    def set_camera_params(self, motion_params):
        # renderers = self.interactorStyle.parent.GetRenderWindow().GetRenderers()
        cam = self.ren.GetActiveCamera()

        self.apply_rotation_to_cam(cam, motion_params)
        self.apply_shifting_to_cam(cam, motion_params)

        self.ren.ResetCamera()
        self.ren.ResetCameraClippingRange()
        self.vtkWidget.Initialize()

    def apply_shifting_to_cam(self, camera, motion_params):
        t_ax = motion_params[0]
        t_cor = motion_params[1]
        t_sag = motion_params[2]

        reference_position = self.interactorStyle.reference_position
        current_cor = -(t_cor - reference_position[0])
        current_sag = - (t_sag - reference_position[1])
        current_ax = - (t_ax - reference_position[2])
        camera.SetPosition(current_cor, current_sag, current_ax)
        #print("Position(apply shifting to cam): " + str(camera.GetPosition()))

    def apply_rotation_to_cam(self, camera, motion_params):
        orientation = ([motion_params[5], motion_params[3], motion_params[4]])
        focus = np.zeros(3)
        position = self.interactorStyle.reference_position
        viewup = np.zeros(3)

        focus[0] = position[0] - -cos(radians(orientation[0])) * sin(radians(orientation[1]))
        focus[1] = position[1] - sin(radians(orientation[0]))
        focus[2] = position[2] - cos(radians(orientation[0])) * cos(radians(orientation[1]))

        viewup[0] = cos(radians(orientation[1])) * sin(radians(orientation[2])) + sin(radians(orientation[1])) * sin(
            radians(orientation[2])) * cos(radians(
            orientation[2]))
        viewup[1] = cos(radians(orientation[0])) * cos(radians(orientation[2]))
        viewup[2] = sin(radians(orientation[1])) * sin(radians(orientation[2])) - cos(radians(orientation[1])) * sin(
            radians(orientation[0])) * cos(radians(
            orientation[2]))

        camera.SetViewUp(viewup)
        camera.SetFocalPoint(focus)
        # print("Orientation (apply rotation): " + str(camera.GetOrientation()))


def set_rotation(self, rotation):
    rotMat = help_functions.get_Rt(rotation)

    transform = vtk.vtkTransform()
    transform.Identity()
    matrix = help_functions.GetVTKMatrix(rotMat)
    transform.Concatenate(matrix)

    # transformFilter = vtk.vtkTransformPolyDataFilter()
    # transformFilter.SetTransform(transform)
    # transformFilter.SetInputConnection(self.arrowSource.GetOutputPort())
    # transformFilter.Update()

    # coneMapper = vtk.vtkPolyDataMapper()
    # coneMapper.SetInputConnection(transformFilter.GetOutputPort())

    # actor = vtk.vtkActor()
    # actor.SetMapper(coneMapper)

    self.actor.SetUserTransform(transform)
    self.ren.ResetCameraClippingRange()
    self.vtkWidget.Initialize()
    self.iren.Initialize()
    # enable user interface interactor
    # iren.Initialize()
    # renWin.Render()
    # iren.Start()
    # print("try to set rotation")

#  def init_interactor(self):
#      self.interactorStyle.onCustomInitialization()
#      a = 10
