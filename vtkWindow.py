import numpy as np
import vtk
from math import *
from PyQt5 import QtWidgets
from dependency.InteractorStyle import InteractorStyle
from dependency.RenderWindowInteractor import *
from include import help_functions


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
        self.reference_position = self.initial_camera.GetPosition()
        self.reference_motion_params = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def init_camera(self):
        initial_camera = vtk.vtkCamera()
        initial_camera.DeepCopy(self.initial_camera)
        self.ren.SetActiveCamera(initial_camera)
        self.iren.Initialize()

    def set_camera_params(self, motion_params, rotation_bool, shifting_bool):
        camera = self.ren.GetActiveCamera()
        if (motion_params == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).all():
            self.reference_motion_params = motion_params[:]
            self.init_camera()
            return
        if (rotation_bool == True):
            self.apply_rotation_to_cam(camera, motion_params)
        if (shifting_bool == True):
            self.apply_movement_to_cam(camera, motion_params)

        # print("Focal (VTK): " + str(camera.GetFocalPoint()))
        # print("View Up (VTK): " + str(camera.GetViewUp()))
        # print("Motion params: " + str(motion_params))
        # print("Position(VTK): " + str(camera.GetPosition()))
        # print("Reference motion params: " + str(self.reference_motion_params))
        # print("Orientation(VTK) " + str(camera.GetOrientation()) + "\n")

        self.ren.ResetCamera()
        self.ren.ResetCameraClippingRange()
        self.vtkWidget.Initialize()
        self.iren.Initialize()

    def apply_movement_to_cam(self, camera, motion_params):
        ref = (self.reference_motion_params[0:3])
        current = (motion_params[0:3])

        # compute the new position
        current_cor = self.reference_position[0] - (current[1] - ref[1])
        current_sag = self.reference_position[1] - (current[2] - ref[2])
        current_ax = self.reference_position[2] - (current[0] - ref[0])
        new_position = (current_cor, current_sag, current_ax)

        # Setting camera values
        camera.SetPosition(new_position)
        focal = camera.GetFocalPoint()
        camera.SetFocalPoint(new_position[0], new_position[1], focal[2])
        camera.SetDistance(new_position[2])

        # update new reference motion parameters and reference position
        self.reference_motion_params = motion_params[:]
        self.reference_position = new_position[:]

    def apply_rotation_to_cam(self, camera, motion_params):
        # orientation determined of slider values
        orientation = (motion_params[5], -motion_params[3], motion_params[4])
        # current orientation of camera
        cam_orientation = camera.GetOrientation()

        camera.Elevation(orientation[0] - cam_orientation[0])
        camera.Azimuth(orientation[1] + cam_orientation[1])
        camera.Roll(orientation[2] - cam_orientation[2])

        self.reference_motion_params = motion_params[:]

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
