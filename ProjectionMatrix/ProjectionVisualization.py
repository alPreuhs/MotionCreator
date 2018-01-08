import math
import random

import numpy as np
import vtk

from Math import projection
from Math import ProjectiveGeometry as pg


def get_axis_label_actor(text, position, ren):
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
    ren.AddActor(textActor)
    return textActor


def get_detector_edge_points(p, sid, size_u, size_v):
    p_inv = np.linalg.pinv(p)
    plane1 = pg.plane_p3(p[0, :])
    plane2 = pg.plane_p3(p[1, :])
    plane3 = pg.plane_p3(p[2, :])
    source_pos = (plane1.meet(plane2)).meet(plane3)
    plane_dtor = plane3.get_plane_at_distance(sid)
    #### Calculate the corner Points of Detector
    bp00 = pg.point_p2(0, 0).backproject(p_inv)
    bp01 = pg.point_p2(0, size_v).backproject(p_inv)
    bp10 = pg.point_p2(size_u, 0).backproject(p_inv)
    bp11 = pg.point_p2(size_u, size_v).backproject(p_inv)
    p00 = (bp00.join(source_pos)).meet(plane_dtor)
    p01 = (bp01.join(source_pos)).meet(plane_dtor)
    p10 = (bp10.join(source_pos)).meet(plane_dtor)
    p11 = (bp11.join(source_pos)).meet(plane_dtor)
    return (p00, p01, p10, p11)


def add_detector_frame(edges, ren, jpgfile=None):
    # define color
    black = [0, 0, 0]
    # extract edge points
    p00 = edges[0].get_euclidean_point()
    p01 = edges[1].get_euclidean_point()
    p10 = edges[2].get_euclidean_point()
    p11 = edges[3].get_euclidean_point()
    # converte points to vtkPoints
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(4)
    points.SetPoint(0, p00[0], p00[1], p00[2])
    points.SetPoint(1, p01[0], p01[1], p01[2])
    points.SetPoint(2, p10[0], p10[1], p10[2])
    points.SetPoint(3, p11[0], p11[1], p11[2])
    # draw lines between vtkPoints
    lines = vtk.vtkCellArray()
    lines.InsertNextCell(5)
    lines.InsertCellPoint(0)
    lines.InsertCellPoint(1)
    lines.InsertCellPoint(3)
    lines.InsertCellPoint(2)
    lines.InsertCellPoint(0)
    # join points and lines within vtkpolydata
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    # assign color to every point
    colors = vtk.vtkUnsignedCharArray()
    colors.SetNumberOfComponents(3)
    colors.SetName("Colors")
    colors.InsertNextTupleValue(black)
    colors.InsertNextTupleValue(black)
    colors.InsertNextTupleValue(black)
    colors.InsertNextTupleValue(black)
    polygon.GetPointData().SetScalars(colors)
    polygon.Modified()
    # map polydata
    if jpgfile is not None:
        p00m = np.matrix(p00).T
        p01m = np.matrix(p01).T
        p10m = np.matrix(p10).T
        p11m = np.matrix(p11).T
        v1 = (p00m - p01m)
        v2 = (p00m - p10m)
        normal = np.cross(v1.T, v2.T)
        d1 = np.linalg.norm(p00m - p01m)
        p10m = np.matrix(p10).T
        d2 = np.linalg.norm(p00m - p10m)
        plane = vtk.vtkPlaneSource()
        cp = ((p00m + p01m + p10m + p11m) / 4)

        plane.SetPoint1(v2[0], v2[1], v2[2])
        plane.SetPoint2(v1[0], v1[1], v1[2])
        plane.SetCenter(cp[0], cp[1], cp[2])
        plane.SetNormal(-normal[0, 0], -normal[0, 1], -normal[0, 2])
        print('asd')
        # Read the image data from a file
        reader = vtk.vtkPNGReader()
        reader.SetFileName(jpgfile)
        # Create texture object
        texture = vtk.vtkTexture()
        texture.SetInputConnection(reader.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(plane.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetTexture(texture)
        ren.AddActor(actor)

    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    # create actor and add polydatamapper to actor
    polygonActor = vtk.vtkActor()
    polygonActor.SetMapper(polygonMapper)
    if jpgfile is not None:
        polygonActor.SetTexture(texture)

    polygonActor.GetProperty().LightingOff()
    # polygonActor.GetProperty().SetLineWidth(1.0)
    polygonMapper.Update()
    ren.AddActor(polygonActor)


def add_source_pos(source_pos, ren, color=(0.0, 0.0, 0.0)):
    cord = source_pos.get_euclidean_point()
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetCenter(cord[0], cord[1], cord[2])
    sphereSource.SetRadius(10)
    sphere_mapper = vtk.vtkPolyDataMapper()
    sphere_mapper.SetInputConnection(sphereSource.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(sphere_mapper)
    actor.GetProperty().SetColor(color[0], color[1], color[2])
    ren.AddActor(actor)
    return actor


def add_cone_edges(source_pos, edges, ren):
    light_white = [0, 0, 0]
    lighter_white = [240, 240, 240]
    p00 = edges[0].get_euclidean_point()
    p01 = edges[1].get_euclidean_point()
    p10 = edges[2].get_euclidean_point()
    p11 = edges[3].get_euclidean_point()
    sp = source_pos.get_euclidean_point()

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(5)
    points.SetPoint(0, p00[0], p00[1], p00[2])
    points.SetPoint(1, p01[0], p01[1], p01[2])
    points.SetPoint(2, p10[0], p10[1], p10[2])
    points.SetPoint(3, p11[0], p11[1], p11[2])
    points.SetPoint(4, sp[0], sp[1], sp[2])

    lines = vtk.vtkCellArray()
    lines.InsertNextCell(7)
    lines.InsertCellPoint(0)
    lines.InsertCellPoint(4)
    lines.InsertCellPoint(1)
    lines.InsertCellPoint(4)
    lines.InsertCellPoint(2)
    lines.InsertCellPoint(4)
    lines.InsertCellPoint(3)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    polygonActor = vtk.vtkActor()
    polygonActor.SetMapper(polygonMapper)
    polygonActor.GetProperty().SetColor(0.8, 0.8, 0.8)
    return polygonActor


def add_coord(endPos, color, ren):
    arrowSource = vtk.vtkArrowSource()
    arrowSource.SetShaftRadius(0.01)
    arrowSource.SetTipLength(0.1)
    arrowSource.SetTipRadius(0.03)

    startPoint = [0 for i in range(3)]
    startPoint[0] = 0
    startPoint[1] = 0
    startPoint[2] = 0
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
    ##alternatively to using the transform above one could use: actor.RotateZ etc and Scale


    actor.GetProperty().SetColor(color[0], color[1], color[2])

    ren.AddActor(actor)


def drawRadianPlane(plane, origin, startpos, radians, num_vertices, ren, to_much):
    points_on_radian = []
    H = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    ###not sure if next step are neccessary...todo: think about it
    orig_mat = np.matrix(origin.get_flat()).T
    orig_mat = orig_mat / orig_mat[-1]
    sp_mat = np.matrix(startpos.get_flat()).T
    sp_mat = sp_mat / sp_mat[-1]

    H[0:3, 3] = -orig_mat[0:3]
    H_inv = np.linalg.inv(H)
    rot_axis = np.matrix(plane.get_hesse_form()[0:3]).T

    points = vtk.vtkPoints()
    points.InsertPoint(0, 0, 0, 0)
    points.InsertPoint(1, 0, 0, 1)
    points.InsertPoint(2, 0, 1, 1)
    points.InsertPoint(3, 0, 1, 0)

    for i in range(0, num_vertices + 2):
        rad = i * radians / (num_vertices + 1)
        R = projection.get_rotation_matrix_by_axis_and_angle(rot_axis, math.degrees(rad), True)
        points_on_radian.append((H_inv * R * H * sp_mat)[0:3, :])

    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len(points_on_radian) + 2)

    polygon = vtk.vtkPolygon()
    polygon.GetPointIds().SetNumberOfIds(len(points_on_radian) + 2)

    points.SetPoint(0, orig_mat[0, 0], orig_mat[1, 0], orig_mat[2, 0])
    polygon.GetPointIds().SetId(0, 0)
    for i in range(len(points_on_radian)):
        points.SetPoint(i + 1, points_on_radian[i][0, 0], points_on_radian[i][1, 0], points_on_radian[i][2, 0])
        polygon.GetPointIds().SetId(i + 1, i + 1)

    polygon.GetPointIds().SetId(len(points_on_radian) + 1, 0)

    # Add the polygon to a list of polygons
    polygons = vtk.vtkCellArray()
    polygons.InsertNextCell(polygon)

    # Create a PolyData
    polygonPolyData = vtk.vtkPolyData()
    polygonPolyData.SetPoints(points)
    polygonPolyData.SetPolys(polygons)

    # Create a mapper and actor
    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(polygonPolyData)
    else:
        mapper.SetInputData(polygonPolyData)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    if to_much:
        actor.GetProperty().SetColor(0, 0, 1)
    else:
        actor.GetProperty().SetColor(1, 0, 0)
    ren.AddActor(actor)


def drawRadian(plane, origin, startpos, radians, num_vertices, ren):
    points_on_radian = []
    H = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    ###not sure if next step are neccessary...todo: think about it
    orig_mat = np.matrix(origin.get_flat()).T
    orig_mat = orig_mat / orig_mat[-1]
    sp_mat = np.matrix(startpos.get_flat()).T
    sp_mat = sp_mat / sp_mat[-1]
    H[0:3, 3] = -orig_mat[0:3]
    H_inv = np.linalg.inv(H)
    rot_axis = np.matrix(plane.get_hesse_form()[0:3]).T
    for i in range(0, num_vertices + 2):
        rad = i * radians / (num_vertices + 1)
        R = projection.get_rotation_matrix_by_axis_and_angle(rot_axis, math.degrees(rad), True)
        points_on_radian.append((H_inv * R * H * sp_mat)[0:3, :])
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len(points_on_radian))
    for i in range(len(points_on_radian)):
        points.SetPoint(i, points_on_radian[i][0, 0], points_on_radian[i][1, 0], points_on_radian[i][2, 0])
    lines = vtk.vtkCellArray()
    # lines.InsertNextCell(len(points_on_radian)-1)

    lines.InsertNextCell(len(points_on_radian))
    for i in range(len(points_on_radian)):
        lines.InsertCellPoint(i)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    # vtkPolyDataMapper is a class that maps polygonal data (i.e., vtkPolyData)
    # to graphics primitives
    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    polygonActor = vtk.vtkActor()
    polygonActor.SetMapper(polygonMapper)
    polygonActor.GetProperty().SetColor(0, 0, 0)
    # polygonActor.GetProperty().SetLineWidth(5)
    ren.AddActor(polygonActor)


def vis_foreshortening(P, line, sid, point_on_line, size_v, ren, min_vis_degree=4):
    fs_ang = evaluation_methods.calculateForeshortening(0, P, line)
    if (fs_ang < min_vis_degree):
        return
    det_plane = pg.plane_p3(P[2, :]).get_plane_at_distance(sid)
    sp = projection.get_source_position(P)
    angle_origin = det_plane.meet(line)
    det_point = point_on_line.project(P).backproject(np.linalg.pinv(P)).join(sp).meet(det_plane)
    ###Draw line from projection to meet_point
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(2)
    points.SetPoint(0, angle_origin.e()[0], angle_origin.e()[1], angle_origin.e()[2])
    points.SetPoint(1, point_on_line.e()[0], point_on_line.e()[1], point_on_line.e()[2])

    lines = vtk.vtkCellArray()
    lines.InsertNextCell(2)
    lines.InsertCellPoint(0)
    lines.InsertCellPoint(1)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    # vtkPolyDataMapper is a class that maps polygonal data (i.e., vtkPolyData)
    # to graphics primitives
    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    polygonActor = vtk.vtkActor()
    polygonActor.SetMapper(polygonMapper)
    polygonActor.GetProperty().SetColor(0, 0, 0)
    polygonActor.GetProperty().SetOpacity(0.3)
    # polygonActor.GetProperty().SetLineWidth(5)
    ren.AddActor(polygonActor)
    ###Draw line from line to meet_point
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(2)
    points.SetPoint(0, angle_origin.e()[0], angle_origin.e()[1], angle_origin.e()[2])
    points.SetPoint(1, det_point.e()[0], det_point.e()[1], det_point.e()[2])
    lines = vtk.vtkCellArray()
    lines.InsertNextCell(2)
    lines.InsertCellPoint(0)
    lines.InsertCellPoint(1)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    # vtkPolyDataMapper is a class that maps polygonal data (i.e., vtkPolyData)
    # to graphics primitives
    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    polygonActor = vtk.vtkActor()
    polygonActor.SetMapper(polygonMapper)
    polygonActor.GetProperty().SetColor(0, 0, 0)
    # polygonActor.GetProperty().SetLineWidth(5)
    polygonActor.GetProperty().SetOpacity(0.5)
    ren.AddActor(polygonActor)
    dir_det_line = np.matrix(det_point.e()) - np.matrix(angle_origin.e())
    dir_det_line = dir_det_line / np.linalg.norm(dir_det_line)
    start_pos = np.matrix(angle_origin.e()) + 100 * dir_det_line
    if (angle_origin.project(P).e()[1] > size_v / 2):
        sign = -1
    else:
        sign = 1
    drawRadian(line.join(sp), angle_origin, pg.point_p3(start_pos), sign * math.radians(fs_ang), 30, ren)


def create_basic_scene(p, sid, size_u, size_v, ren, jpgfile=None, opacity=1):
    source_pos = projection.get_source_position(p)
    det_corner = get_detector_edge_points(p, sid, size_u, size_v)
    add_source_pos(source_pos, ren, color=(1, 1, 1))
    add_cone_edges(source_pos, det_corner, ren)
    add_detector_frame(det_corner, ren, jpgfile=jpgfile)
    add_coord((100, 0, 0), (0, 0, 255), ren)
    add_coord((0, 100, 0), (255, 0, 0), ren)
    add_coord((0, 0, 100), (0, 0, 0), ren)
    get_axis_label_actor("x", (100, 0, 0), ren)
    get_axis_label_actor("y", (0, 100, 0), ren)
    get_axis_label_actor("z", (0, 0, 100), ren)


def create_line_and_projection(line, projMatrix, sid, size_u, size_v, p1, p2, ren, createProjectionPlane=False):
    sourcePos = projection.get_source_position(projMatrix)
    det_plane = pg.plane_p3(projMatrix[2, :]).get_plane_at_distance(sid)
    det_Corner = get_detector_edge_points(projMatrix, sid, size_u, size_v)
    line_proj = line.project(projMatrix)
    ###########Draw line on Detector
    line_up = pg.point_p2(0, 0).join(pg.point_p2(0, size_v))
    line_right = pg.point_p2(0, size_v).join(pg.point_p2(size_u, size_v))
    line_left = pg.point_p2(0, 0).join(pg.point_p2(size_u, 0))
    line_down = pg.point_p2(size_u, 0).join(pg.point_p2(size_u, size_v))
    ###extract the two points that cross the detector
    delta = 0.1
    pt_list = [line_proj.meet(line_up), line_proj.meet(line_right), line_proj.meet(line_left),
               line_proj.meet(line_down)]
    det_meet_points = []
    for point in pt_list:
        if point.get_euclidean_point()[0] > 0 - delta and point.get_euclidean_point()[0] < size_u + delta:
            if point.get_euclidean_point()[1] > 0 - delta and point.get_euclidean_point()[1] < size_v + delta:
                det_meet_points.append(point)
    p1_euclid = p1.get_euclidean_point()
    p2_euclid = p2.get_euclidean_point()
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetCenter(p1_euclid[0], p1_euclid[1], p1_euclid[2])
    sphereSource.SetRadius(10)
    sphere_mapper = vtk.vtkPolyDataMapper()
    sphere_mapper.SetInputConnection(sphereSource.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(sphere_mapper)
    actor.GetProperty().SetColor(1, 0.0, 0.0)
    ren.AddActor(actor)
    p1_euclid = p1.get_euclidean_point()
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetCenter(p2_euclid[0], p2_euclid[1], p2_euclid[2])
    # sphereSource.SetCenter(0, 0, 0)
    sphereSource.SetRadius(10)
    sphere_mapper = vtk.vtkPolyDataMapper()
    sphere_mapper.SetInputConnection(sphereSource.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(sphere_mapper)
    actor.GetProperty().SetColor(1, 0.0, 0.0)
    ren.AddActor(actor)


    if (len(det_meet_points) < 2):
        print('object is outside of projector. choose a more centered line')
        return
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(2)
    bpray_1 = det_meet_points[0].backproject(np.linalg.pinv(projMatrix)).join(sourcePos)
    p3D_1 = bpray_1.meet(det_plane)
    points.SetPoint(0, p3D_1.get_euclidean_point()[0], p3D_1.get_euclidean_point()[1], p3D_1.get_euclidean_point()[2])
    bpray_2 = det_meet_points[1].backproject(np.linalg.pinv(projMatrix)).join(sourcePos)
    p3D_2 = bpray_2.meet(det_plane)
    points.SetPoint(1, p3D_2.get_euclidean_point()[0], p3D_2.get_euclidean_point()[1], p3D_2.get_euclidean_point()[2])
    lines = vtk.vtkCellArray()
    lines.InsertNextCell(2)
    lines.InsertCellPoint(0)
    lines.InsertCellPoint(1)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    # create actor and add polydatamapper to actor
    det_line_actor = vtk.vtkActor()
    det_line_actor.SetMapper(polygonMapper)
    # det_line_actor.GetProperty().SetColor(0.3, 0.3, 0.3)
    det_line_actor.GetProperty().SetColor(1, 1, 1)
    det_line_actor.GetProperty().SetLineWidth(0.5)
    ren.AddActor(det_line_actor)
    #######Add Line in Space
    p1_euclid = p1.get_euclidean_point()
    p2_euclid = p2.get_euclidean_point()
    ###Now add the actual line
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(2)
    points.SetPoint(0, p1_euclid[0], p1_euclid[1], p1_euclid[2])
    points.SetPoint(1, p2_euclid[0], p2_euclid[1], p2_euclid[2])
    lines = vtk.vtkCellArray()
    lines.InsertNextCell(2)
    lines.InsertCellPoint(0)
    lines.InsertCellPoint(1)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    # create actor and add polydatamapper to actor
    line_actor = vtk.vtkActor()
    line_actor.SetMapper(polygonMapper)
    line_actor.GetProperty().SetColor(0, 0, 0)
    line_actor.GetProperty().SetLineWidth(2.5)
    ren.AddActor(line_actor)

    #######Add Projected Line in Space
    p1_det = p1.project(projMatrix).backproject(np.linalg.pinv(projMatrix)).join(sourcePos).meet(det_plane)
    p2_det = p2.project(projMatrix).backproject(np.linalg.pinv(projMatrix)).join(sourcePos).meet(det_plane)
    ###Now add the actual line
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(2)
    points.SetPoint(0, p1_det.get_euclidean_point()[0], p1_det.get_euclidean_point()[1],
                    p1_det.get_euclidean_point()[2])
    points.SetPoint(1, p2_det.get_euclidean_point()[0], p2_det.get_euclidean_point()[1],
                    p2_det.get_euclidean_point()[2])
    lines = vtk.vtkCellArray()
    lines.InsertNextCell(2)
    lines.InsertCellPoint(0)
    lines.InsertCellPoint(1)
    polygon = vtk.vtkPolyData()
    polygon.SetPoints(points)
    polygon.SetLines(lines)
    polygonMapper = vtk.vtkPolyDataMapper()
    polygonMapper.SetInputData(polygon)
    polygonMapper.Update()
    # create actor and add polydatamapper to actor
    line_actor = vtk.vtkActor()
    line_actor.SetMapper(polygonMapper)
    line_actor.GetProperty().SetColor(0, 0, 0)
    ren.AddActor(line_actor)

    ####draw plane
    plane_line_sourcePos = line.join(sourcePos)
    points = vtk.vtkPoints()
    triangles = vtk.vtkCellArray()
    points.SetNumberOfPoints(3)
    points.SetPoint(0, p3D_1.get_euclidean_point()[0], p3D_1.get_euclidean_point()[1], p3D_1.get_euclidean_point()[2])
    points.SetPoint(1, p3D_2.get_euclidean_point()[0], p3D_2.get_euclidean_point()[1], p3D_2.get_euclidean_point()[2])
    points.SetPoint(2, sourcePos.get_euclidean_point()[0], sourcePos.get_euclidean_point()[1],
                    sourcePos.get_euclidean_point()[2])
    triangle = vtk.vtkTriangle();
    triangle.GetPointIds().SetId(0, 0);
    triangle.GetPointIds().SetId(1, 1);
    triangle.GetPointIds().SetId(2, 2);
    triangles.InsertNextCell(triangle);

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(triangles)
    # mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    # actor
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0, 0, 0)
    actor.GetProperty().SetOpacity(0.3)
    ren.AddActor(actor)
