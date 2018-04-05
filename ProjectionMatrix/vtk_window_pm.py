import numpy as np
import vtk
import random
from PyQt5 import QtWidgets
from dependency.InteractorStyle import InteractorStyle
from dependency.RenderWindowInteractor import *



class vtk_window_pm():
    def vtkWidget(self, qtFrame):
        self.vl = QtWidgets.QGridLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(qtFrame)

        self.vl.addWidget(self.vtkWidget)
        self.vl.setContentsMargins(0, 0, 0, 0)

        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)

        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        self.actor = vtk.vtkActor()

        self.ren.SetBackground(1, 1, 1)
        textActor = self.get_axis_label_actor('x', [50, 0, 0], self.ren)
        self.ren.AddActor(textActor)
        textActor = self.get_axis_label_actor('y', [0, 50, 0], self.ren)
        self.ren.AddActor(textActor)
        textActor = self.get_axis_label_actor('z', [0, 0, 50], self.ren)
        self.ren.AddActor(textActor)
        self.add_coord([50, 0, 0], [0, 0, 0], self.ren, shatRadius=0.02, tipLength=0.1, tipRadius=0.05)
        self.add_coord([0, 50, 0], [0, 0, 0], self.ren, shatRadius=0.02, tipLength=0.1, tipRadius=0.05)
        self.add_coord([0, 0, 50], [0, 0, 0], self.ren, shatRadius=0.02, tipLength=0.1, tipRadius=0.05)
        self.ren.ResetCamera()
        self.vtkWidget.Initialize()
        self.iren.Initialize()

        # Set the InteractorStyle to self written Interactor style, where we can
        # access the signals (i think .AddObserver("...event..., func") would do the same...)

        self.interactorStyle = InteractorStyle( self, parent=self.iren)
        self.iren.SetInteractorStyle(self.interactorStyle)
        qtFrame.setLayout(self.vl)

    def test(self):
        print("I do nothing")

    def add_actors(self, actor, color=[0, 0, 0]):
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        self.ren.AddActor(actor)

    def update(self):
        self.ren.ResetCamera()
        self.ren.ResetCameraClippingRange()
        self.vtkWidget.Initialize()
        self.iren.Initialize()

    def remove_actor(self, actor):
        self.ren.RemoveActor(actor);

    def get_axis_label_actor(self, text, position, ren):
        atext = vtk.vtkVectorText()
        atext.SetText(text)
        textMapper = vtk.vtkPolyDataMapper()
        textMapper.SetInputConnection(atext.GetOutputPort())
        textActor = vtk.vtkFollower()
        textActor.SetMapper(textMapper)
        textActor.SetScale(20, 20, 20)
        textActor.AddPosition(position[0], position[1], position[2])
        textActor.GetProperty().SetColor(0, 0, 0)
        textActor.SetCamera(ren.GetActiveCamera())
        return textActor

    def add_coord(self, endPos, color, ren, startpos=(0, 0, 0), shatRadius=0.005, tipLength=0.05, tipRadius=0.015):
        arrowSource = vtk.vtkArrowSource()
        arrowSource.SetShaftRadius(shatRadius)
        arrowSource.SetTipLength(tipLength)
        arrowSource.SetTipRadius(tipRadius)
        arrowSource.SetShaftResolution(50)
        arrowSource.SetTipResolution(50)

        startPoint = [0 for i in range(3)]
        startPoint[0] = startpos[0]
        startPoint[1] = startpos[1]
        startPoint[2] = startpos[2]
        endPoint = [0 for i in range(3)]
        endPoint[0] = endPos[0]
        endPoint[1] = endPos[1]
        endPoint[2] = endPos[2]

        # Compute a basis
        normalizedX = [0 for i in range(3)]
        normalizedY = [0 for i in range(3)]
        normalizedZ = [0 for i in range(3)]

        # The X axis is a vector from start to end
        math = vtk.vtkMath()
        math.Subtract(endPoint, startPoint, normalizedX)
        length = math.Norm(normalizedX)
        math.Normalize(normalizedX)

        # The Z axis is an arbitrary vector cross X
        arbitrary = [0 for i in range(3)]
        arbitrary[0] = random.uniform(-10, 10)
        arbitrary[1] = random.uniform(-10, 10)
        arbitrary[2] = random.uniform(-10, 10)
        math.Cross(normalizedX, arbitrary, normalizedZ)
        math.Normalize(normalizedZ)

        # The Y axis is Z cross X
        math.Cross(normalizedZ, normalizedX, normalizedY)
        matrix = vtk.vtkMatrix4x4()

        # Create the direction cosine matrix
        matrix.Identity()
        for i in range(3):
            matrix.SetElement(i, 0, normalizedX[i])
            matrix.SetElement(i, 1, normalizedY[i])
            matrix.SetElement(i, 2, normalizedZ[i])

        # Apply the transforms
        transform = vtk.vtkTransform()
        transform.Translate(startPoint)
        transform.Concatenate(matrix)
        transform.Scale(length, length, length)

        # Transform the polydata
        transformPD = vtk.vtkTransformPolyDataFilter()
        transformPD.SetTransform(transform)
        transformPD.SetInputConnection(arrowSource.GetOutputPort())

        # Create a mapper and actor for the arrow
        mapper = vtk.vtkPolyDataMapper()
        actor = vtk.vtkActor()
        mapper.SetInputConnection(transformPD.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().LightingOff()
        ##alternatively to using the transform above one could use: actor.RotateZ etc and Scale
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        ren.AddActor(actor)


