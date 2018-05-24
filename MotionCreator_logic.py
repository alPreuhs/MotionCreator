import math
import os.path
import sys
import xml.etree.ElementTree as etree
import PyQt5
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import os
from ProjectionMatrix.ProjMatCreator_logic import ProjMatCreator_logic
from include import akima
from include import help_functions
from include import readWriteRaw
from motionCreator_gui import Ui_MotionCreator
from vtkWindow import vtkWindow


class MotionCreator_logic(Ui_MotionCreator):
    def __init__(self, MainWindow):
        self.MainWindow = MainWindow

        # tracback is disabled by default,
        # the following reactivats it
        def excepthook(type_, value, traceback_):
            traceback.print_exception(type_, value, traceback_)
            QtCore.qFatal('')

        sys.excepthook = excepthook
        self.read_xml()
        ##Setup UI, which is created via qt designer and pyuic5
        Ui_MotionCreator.__init__(self)
        self.setupUi(self.MainWindow)
        self.MainWindow.resized.connect(self.resizeEvent)
        # self.connect(senderObject, QtCore.SIGNAL("signalName()"), self.slotMethod)
        ##Setup the VTK window
        self.vtk_handle = vtkWindow()
        self.vtk_handle.vtkWidget(self.view3D, self)
        #        self.vtk_handle.init_interactor()
        ##define Variables
        self.init_member_variables()
        ##Connect Slider logic
        self.connect_slider()
        self.connect_external_window_buttons()
        self.connect_buttons()
        self.connect_checkbox()
        self.gs = QtWidgets.QGraphicsScene()
        self.col = QtGui.QColor(0, 255, 0)
        self.pen = QtGui.QPen()
        self.pen.setWidth(5)
        co = QtGui.QColor(0, 0, 255)
        self.pen.setColor(co)
        self.setup_mpr()
        self.resizeEvent()
        ###load default biplane projection matrix
        self.proj_mat_creator.bt_confirm_proj_mat.click()

    def resizeEvent(self):
        self.ViewAxial.fitInView(self.gps_axial_placeholder.boundingRect(), QtCore.Qt.KeepAspectRatio)
        self.ViewSagittal.fitInView(self.gps_sagittal_placeholder.boundingRect(), QtCore.Qt.KeepAspectRatio)
        self.ViewCoronal.fitInView(self.gps_coronal_placeholder.boundingRect(), QtCore.Qt.KeepAspectRatio)

    def setup_mpr(self):
        img_coronal = QtGui.QImage('include//coronal.png')
        img_axial = QtGui.QImage('include//axial.png')
        img_sagittal = QtGui.QImage('include//sagittal.png')
        ###wrap QImage to Pixmap and create a default QGraphicsPixmapItem that will be used
        ###as resize reference (such that a rotation of the actual pixmap will not affect the size)
        self.pix_coronal = QtGui.QPixmap(img_coronal)
        self.gps_coronal_placeholder = QtWidgets.QGraphicsPixmapItem(self.pix_coronal)
        self.pix_axial = QtGui.QPixmap(img_axial)
        self.gps_axial_placeholder = QtWidgets.QGraphicsPixmapItem(self.pix_axial)
        self.pix_sagittal = QtGui.QPixmap(img_sagittal)
        self.gps_sagittal_placeholder = QtWidgets.QGraphicsPixmapItem(self.pix_sagittal)
        ## define the offset which is the negative position of each pixmap, such that ensure for the
        ## correct rotation axis (default would be top left of the image)
        self.offset = np.zeros(shape=(3, 2))
        self.offset[0, ...] = [-self.pix_coronal.width() / 2, -self.pix_coronal.height() * 3 / 4]
        self.offset[1, ...] = [-self.pix_axial.width() / 2, -self.pix_axial.height() * 2 / 5]
        self.offset[2, ...] = [-self.pix_sagittal.width() / 2, -self.pix_sagittal.height() * 3 / 4]
        self.gps_coronal = QtWidgets.QGraphicsPixmapItem(self.pix_coronal)
        self.gps_coronal.setOffset(self.offset[0, 0], self.offset[0, 1])
        self.gps_coronal.setPos(-  self.offset[0, 0], -self.offset[0, 1])
        self.gps_axial = QtWidgets.QGraphicsPixmapItem(self.pix_axial)
        self.gps_axial.setOffset(self.offset[1, 0], self.offset[1, 1])
        self.gps_axial.setPos(-  self.offset[1, 0], -self.offset[1, 1])
        self.gps_sagittal = QtWidgets.QGraphicsPixmapItem(self.pix_sagittal)
        self.gps_sagittal.setOffset(self.offset[2, 0], self.offset[2, 1])
        self.gps_sagittal.setPos(-  self.offset[2, 0], -self.offset[2, 1])
        self.gs_coronal = QtWidgets.QGraphicsScene()
        self.gs_axial = QtWidgets.QGraphicsScene()
        self.gs_sagittal = QtWidgets.QGraphicsScene()
        self.gs_coronal.addItem(self.gps_coronal)
        self.gs_axial.addItem(self.gps_axial)
        self.gs_sagittal.addItem(self.gps_sagittal)
        self.ViewCoronal.setScene(self.gs_coronal)
        self.ViewAxial.setScene(self.gs_axial)
        self.ViewSagittal.setScene(self.gs_sagittal)

    def read_xml(self):

        if os.path.exists('include//settings.xml'):
            tree = etree.parse('include//settings.xml')
            ####use this to generate new entries
            # node = SubElement(tree.getroot(),'pMatDir')
            # node.text=r'C:\someFolder'
            # tree.write("settings.xml")
            root = tree.getroot()
            for child in root:
                if child.tag == 'pMatDir':
                    if child.text != None and child.text != ' ':
                        self.p_mat_dir = child.text
                    else:
                        self.p_mat_dir = 'c:\\'
                elif child.tag == 'useThisProjMatrix':
                    if child.text != None and child.text != ' ':
                        self.proj_mat_file_is_set = True
                        self.p_mat_dir = child.text
                        self.load_p_mat()
        else:
            self.create_xml()

    def create_xml(self):
        root = etree.Element('data')
        xmltree = etree.ElementTree(element=root)
        doc = etree.SubElement(root, 'pMatDir')
        doc.text = ' '
        doc2 = etree.SubElement(root, 'useThisProjMatrix')
        doc2.text = ' '
        xmltree.write("include//settings.xml")
        ###now we created the xml file --> following we need to call the xml read routine
        self.read_xml()

    def connect_checkbox(self):
        self.cb_cosinus.clicked.connect(self.on_cb_enable_cos_interpolation)
        self.cb_akima.clicked.connect(self.on_cb_enable_akima_interpolation)

    def on_cb_enable_akima_interpolation(self):
        self.used_interpolation = self.akima_interp

    def on_cb_enable_cos_interpolation(self):
        self.used_interpolation = self.cos_interp

    def init_member_variables(self):
        ##initialize projection matrix creator
        self.proj_mat_creator_widget = QtWidgets.QWidget()
        self.proj_mat_creator = ProjMatCreator_logic(self.proj_mat_creator_widget)
        self.akima_interp = 0
        self.cos_interp = 1
        self.used_interpolation = self.cos_interp
        self.num_proj = int(self.proj_mat_creator.tf_num_proj.text()) + 1
        self.current_proj = 0
        self.set_text_selected_timepoint_label()
        self.used_projections = []
        self.lb_proj_is_used.setAutoFillBackground(True)  # To enable background colouring
        self.set_motion_label_and_btn()
        self.motion_parameters = np.zeros((self.num_proj, 6), dtype=np.float32)
        self.cam_motion_parameters = np.zeros((self.num_proj, 6), dtype=np.float32)
        self.slider_motion_parameters = np.zeros((self.num_proj, 6), dtype=np.float32)
        # self.cam_reset_motion_parameters = np.zeros((self.num_proj, 6), dtype=np.float32)
        self.p_mat = []
        self.motion_from_vtk = False

    def set_text_selected_timepoint_label(self):
        self.lb_Time_ScrollBar.setText("Selected Timepoint: {}/{}".format(self.current_proj + 1, self.num_proj))

    def vtk_motion_to_graphics_view(self, cam_motion_parameters, rotation, shifting):
        t_ax = cam_motion_parameters[0]
        t_cor = cam_motion_parameters[1]
        t_sag = cam_motion_parameters[2]
        r_ax = cam_motion_parameters[3]
        r_cor = cam_motion_parameters[4]
        r_sag = cam_motion_parameters[5]

        self.motion_from_vtk = True
        if rotation == True:
            self.cam_motion_parameters[self.current_proj, 3:6] = cam_motion_parameters[3:6]
            self.on_slider_changed_R_ax(r_ax)
            self.on_slider_changed_R_cor(r_cor)
            self.on_slider_changed_R_sag(r_sag)

        if shifting == True:
            self.cam_motion_parameters[self.current_proj, 0:3] = cam_motion_parameters[0:3]
            self.on_slider_changed_t_ax(t_ax)  # / 2)
            self.on_slider_changed_t_cor(t_cor)
            self.on_slider_changed_t_sag(t_sag)  # / 1.5)
        self.motion_from_vtk = False

    def connect_slider(self):
        ##Connect translation Slider
        self.TSliderAxial.valueChanged.connect(self.on_slider_changed_t_ax)
        self.TSliderCoronal.valueChanged.connect(self.on_slider_changed_t_cor)
        self.TSliderSagittal.valueChanged.connect(self.on_slider_changed_t_sag)
        ##Connect Rotation Slider
        self.RSliderAxial.valueChanged.connect(self.on_slider_changed_R_ax)
        self.RSliderCoronal.valueChanged.connect(self.on_slider_changed_R_cor)
        self.RSliderSagittal.valueChanged.connect(self.on_slider_changed_R_sag)
        ##Connect Time Slider
        self.sb_time_cursor.valueChanged.connect(self.on_time_cursor_changed)

    def on_slider_changed_t_ax(self, i):
        if i is not 0:
            self.gps_axial.setPos(-  self.offset[1, 0], -self.offset[1, 1] + i)
            self.motion_parameters[self.current_proj, 0] = i * 2  ##currently leftwards rightwards
            if self.motion_from_vtk == False:
                self.on_slider_changed()
                # self.slider_motion_parameters[self.current_proj, 0] = i * 2 - self.cam_motion_parameters[
                #     self.current_proj, 0]
            else:
                if self.TSliderAxial.value() == i:
                    return
                self.TSliderAxial.setValue(i)
                self.register_projection()

    def on_slider_changed_t_cor(self, i):
        if i is not 0:
            self.gps_coronal.setPos(-self.offset[0, 0] + i, -self.offset[0, 1])
            self.motion_parameters[self.current_proj, 1] = i  ##currently upwards
            if self.motion_from_vtk == False:
                self.on_slider_changed()
                # self.slider_motion_parameters[self.current_proj, 1] = i - self.cam_motion_parameters[
                #     self.current_proj, 1]
            else:
                if (self.TSliderCoronal.value() == i):
                    return
                self.TSliderCoronal.setValue(i)
                self.register_projection()

    def on_slider_changed_t_sag(self, i):
        if i is not 0:
            self.gps_sagittal.setPos(-  self.offset[2, 0], -self.offset[2, 1] - i)
            self.motion_parameters[self.current_proj, 2] = i * 1.5
            if self.motion_from_vtk == False:
                self.on_slider_changed()
                # self.slider_motion_parameters[self.current_proj, 2] = i * 1.5 + self.cam_motion_parameters[
                #     self.current_proj, 2]
            else:
                if (self.TSliderSagittal.value() == i):
                    return
                self.TSliderSagittal.setValue(i)
                self.register_projection()

    def on_slider_changed_R_ax(self, i):
        if i is not 0:
            self.gps_axial.setRotation(-i)
            self.motion_parameters[self.current_proj, 3] = i
            if self.motion_from_vtk == False:
                self.on_slider_changed()
                # self.slider_motion_parameters[self.current_proj, 3] = i - self.cam_motion_parameters[
                #     self.current_proj, 3]
            else:
                if (self.RSliderAxial.value() == i):
                    return
                self.RSliderAxial.setValue(i)
                self.register_projection()

    def on_slider_changed_R_cor(self, i):
        # self.previous_motion_parameters = self.motion_parameters
        # print(self.previous_motion_parameters[self.current_proj])
        if i is not 0:
            self.gps_coronal.setRotation(-i)
            self.motion_parameters[self.current_proj, 4] = i
            if self.motion_from_vtk == False:
                self.on_slider_changed()
                # self.slider_motion_parameters[self.current_proj, 4] = i - self.cam_motion_parameters[
                #     self.current_proj, 4]
            else:
                if self.RSliderCoronal.value() == i:
                    return
                self.RSliderCoronal.setValue(i)
                self.register_projection()

    def on_slider_changed_R_sag(self, i):
        if i is not 0:
            self.gps_sagittal.setRotation(-i)
            self.motion_parameters[self.current_proj, 5] = i
            if self.motion_from_vtk == False:
                self.on_slider_changed()
                # self.slider_motion_parameters[self.current_proj, 5] = i - self.cam_motion_parameters[
                #     self.current_proj, 5]
            else:
                if self.RSliderSagittal.value() == i:
                    return
                self.RSliderSagittal.setValue(i)
                self.register_projection()

    def on_slider_changed(self):
        self.register_projection()
        self.motion_to_vtk_window()

    def on_time_cursor_changed(self, i):
        self.current_proj = i
        self.set_text_selected_timepoint_label()
        self.set_motion_label_and_btn()
        self.set_sliders_via_array(self.motion_parameters[self.current_proj])
        self.motion_to_vtk_window()
        self.set_mprs()

    def unregister_projection_n(self, num):
        # check if projection was registered
        if num in self.used_projections:
            self.used_projections.remove(num)
        # give user feedback
        self.set_motion_label_and_btn()
        # deactivate respective buttons
        if len(self.used_projections) is 0:
            self.bt_next_proj.setDisabled(True)
            self.bt_prev_proj.setDisabled(True)
            self.bt_reset_all_proj.setDisabled(True)

    def register_projection_n(self, num):
        # activate respective buttons
        if len(self.used_projections) is 0:
            self.bt_next_proj.setEnabled(True)
            self.bt_prev_proj.setEnabled(True)
            self.bt_reset_all_proj.setEnabled(True)
            self.bt_reset_cur_proj.setEnabled(True)
        # check if projection is already register, if not register and sort the list
        if num not in self.used_projections:
            self.used_projections.append(num)
            self.used_projections = sorted(self.used_projections)
            # give user feedback
            self.set_motion_label_and_btn()

    def register_projection(self):
        self.register_projection_n(self.current_proj)

    def set_motion_label_and_btn(self):
        if self.current_proj in self.used_projections:
            ###this is used
            color = QtGui.QColor(51, 255, 51)
            alpha = 140
            values = "{r}, {g}, {b}, {a}".format(r=color.red(), g=color.green(), b=color.blue(), a=alpha)
            self.lb_proj_is_used.setStyleSheet("QLabel { background-color: rgba(" + values + "); }")
            self.lb_proj_is_used.setText("Motion Defined for this Projection")
            self.bt_reset_cur_proj.setEnabled(True)
        else:
            color = QtGui.QColor(247, 84, 84)
            alpha = 140
            values = "{r}, {g}, {b}, {a}".format(r=color.red(), g=color.green(), b=color.blue(), a=alpha)
            self.lb_proj_is_used.setStyleSheet("QLabel { background-color: rgba(" + values + "); }")
            self.lb_proj_is_used.setText("Motion NOT Defined for this Projection")
            self.bt_reset_cur_proj.setDisabled(True)

    def connect_buttons(self):
        # Connect "add" button with a custom function (addInputTextToListbox)
        self.bt_prev_proj.clicked.connect(self.on_bt_prev_proj)
        self.bt_next_proj.clicked.connect(self.on_bt_next_proj)
        self.bt_reset_cur_proj.clicked.connect(self.on_bt_reset_cur_proj)
        self.bt_reset_all_proj.clicked.connect(self.on_bt_reset_all_proj)
        self.bt_interpolate_motion.clicked.connect(self.on_bt_interpolate)
        self.bt_load_pm.clicked.connect(self.on_bt_load_pm)
        self.bt_save.clicked.connect(self.on_bt_save)
        self.bt_save_pm.clicked.connect(self.on_bt_save_pm)
        self.bt_createPM.clicked.connect(self.on_bt_create_pm)
        self.bt_load_motion_file.clicked.connect(self.on_load_motion_file)

    def on_bt_create_pm(self):
        self.proj_mat_creator_widget.show()

    def motion_to_vtk_window(self):
        self.vtk_handle.set_camera_params(self.motion_parameters[self.current_proj])
        #print("Motion params: " + str(self.motion_parameters[self.current_proj]) + "\n")


    def on_bt_interpolate(self):
        if self.used_interpolation is self.cos_interp:
            weight_arg_1 = 0
            for proj_arg in list(self.used_projections):
                if weight_arg_1 < proj_arg:
                    self.cos_weighting(self.motion_parameters[weight_arg_1, ...], self.motion_parameters[proj_arg, ...],
                                       weight_arg_1, weight_arg_1, proj_arg)
                    weight_arg_1 = proj_arg
                else:
                    weight_arg_1 = proj_arg

            if weight_arg_1 < (self.num_proj - 1):
                self.cos_weighting(self.motion_parameters[weight_arg_1, ...],
                                   self.motion_parameters[self.num_proj - 1, ...],
                                   weight_arg_1, weight_arg_1, self.num_proj - 1)

        elif self.used_interpolation is self.akima_interp:
            x = self.used_projections
            if 0 not in x:
                x.append(0)
            if self.num_proj - 1 not in x:
                x.append(self.num_proj - 1)
            x = sorted(x)
            x_vec = np.matrix(x).T * np.matrix([1, 1, 1, 1, 1, 1], dtype=np.float64)
            y_vec = self.motion_parameters[x]

            x_vec_new = np.matrix(np.arange(x_vec[0, 0], x_vec[-1, 0] + 1, 1), dtype=np.float64).T * np.matrix(
                [1, 1, 1, 1, 1, 1], dtype=np.float64)
            y_vec_new = np.zeros(shape=x_vec_new.shape)
            # a = np.vectorize(akima.interpolate)
            for i in range(0, 6):
                ###we need to make some array one-dimensional, currently they are only pseudo-one dimensional i.e. a matrix with nx1 dimension
                x_tmp = np.array(x_vec[..., i])[..., 0]
                y_tmp = np.array(y_vec[..., i])
                x_vec_new_tmp = np.array(x_vec_new[..., i])[..., 0]
                y_vec_new[..., i] = akima.interpolate(x_tmp, y_tmp, x_vec_new_tmp)
            self.motion_parameters = y_vec_new.astype(np.float64)
            for i in range(0, self.num_proj):
                self.register_projection_n(i)

    def cos_weighting(self, weight1, weight2, num, min_val, max_val):
        min_val = float(min_val)
        num = int(num)
        max_val = float(max_val)
        if num > max_val:
            return
        else:
            linear = (num - min_val) / (max_val - min_val)
            cos_weight = -math.cos(linear * math.pi) / 2 + 0.5
            result = weight1 + (cos_weight) * (weight2 - weight1)
            self.motion_parameters[num, ...] = result
            self.register_projection_n(num)
            self.cos_weighting(weight1, weight2, num + 1, min_val, max_val)

    def on_bt_reset_all_proj(self):
        # we need to copy the list, as we remove all the items from the original list
        for num in list(self.used_projections):
            self.reset_projection(num)
        self.reset_routines()

    def on_bt_reset_cur_proj(self):
        self.reset_projection(self.current_proj)
        ##reset projection does not change the sliders...thus we need to do it thereafter
        self.reset_routines()

    def reset_routines(self):
        self.set_sliders_via_array(self.motion_parameters[self.current_proj])
        self.motion_to_vtk_window()
        self.reset_mpr()
        # self.cam_reset_motion_parameters[self.current_proj] = self.cam_motion_parameters[self.current_proj]

    def reset_mpr(self):
        self.gps_axial.setPos(-self.offset[1, 0], -self.offset[1, 1])
        self.gps_coronal.setPos(-self.offset[0, 0], -self.offset[0, 1])
        self.gps_sagittal.setPos(-self.offset[2, 0], -self.offset[2, 1])
        self.gps_axial.setRotation(0)
        self.gps_coronal.setRotation(0)
        self.gps_sagittal.setRotation(0)

    def set_mprs(self):
        ts = self.TSliderSagittal.value()
        tc = self.TSliderCoronal.value()
        ta = self.TSliderAxial.value()
        rs = self.RSliderSagittal.value()
        rc = self.RSliderCoronal.value()
        ra = self.RSliderAxial.value()
        self.gps_axial.setPos(-self.offset[1, 0], -self.offset[1, 1] + ta)
        self.gps_coronal.setPos(-self.offset[0, 0] + tc, -self.offset[0, 1])
        self.gps_sagittal.setPos(-self.offset[2, 0], -self.offset[2, 1] - ts)
        self.gps_axial.setRotation(-ra)
        self.gps_coronal.setRotation(-rc)
        self.gps_sagittal.setRotation(-rs)

    def reset_projection(self, num):
        self.motion_parameters[num, ...] = 0
        self.unregister_projection_n(num)

    ####method to set the sliders manually
    def set_sliders_via_array(self, array_input):
        def set_sliders(ta, tc, ts, ra, rc, rs):
            # block slider signal, as we only want the signal to emit,
            # if we manually change its value (Translation)
            self.TSliderSagittal.blockSignals(True)
            self.TSliderCoronal.blockSignals(True)
            self.TSliderAxial.blockSignals(True)
            # block slider signal, as we only want the signal to emit,
            # if we manually change its value (Rotation)
            self.RSliderSagittal.blockSignals(True)
            self.RSliderCoronal.blockSignals(True)
            self.RSliderAxial.blockSignals(True)
            ##set the actual value (Translation)
            self.TSliderAxial.setValue(ta)
            self.TSliderCoronal.setValue(tc)
            self.TSliderSagittal.setValue(ts)
            ##set the actual value (Rotation)
            self.RSliderAxial.setValue(ra)
            self.RSliderCoronal.setValue(rc)
            self.RSliderSagittal.setValue(rs)
            # Reactivate slider signal (Translation)
            self.TSliderSagittal.blockSignals(False)
            self.TSliderCoronal.blockSignals(False)
            self.TSliderAxial.blockSignals(False)
            # Reactivate slider signal (Rotation)
            self.RSliderSagittal.blockSignals(False)
            self.RSliderCoronal.blockSignals(False)
            self.RSliderAxial.blockSignals(False)

        set_sliders(array_input[0], array_input[1], array_input[2], array_input[3],
                    array_input[4], array_input[5])

    def reset_sliders(self):
        self.TSliderAxial.setValue(0)
        self.TSliderCoronal.setValue(0)
        self.TSliderSagittal.setValue(0)
        self.RSliderAxial.setValue(0)
        self.RSliderCoronal.setValue(0)
        self.RSliderSagittal.setValue(0)

    def on_bt_prev_proj(self):
        ##this is the smallest entry, since all entries are sorted
        prev_proj = self.used_projections[0]
        ##the largest that is small then the current entry will be selected
        for entry in self.used_projections:
            if entry < self.current_proj:
                prev_proj = entry
        self.select_projection(prev_proj)

    def on_bt_next_proj(self):
        ##this is the highest entry, since all entries are sorted
        prev_proj = self.used_projections[-1]
        ##the first value that is greater than the current value is the next balue
        for entry in self.used_projections:
            if entry > self.current_proj:
                self.select_projection(entry)
                return
        self.select_projection(prev_proj)

    def select_projection(self, num):
        self.sb_time_cursor.setValue(num)

    def connect_external_window_buttons(self):
        self.proj_mat_creator.bt_confirm_proj_mat.clicked.connect(self.on_proj_matrix_created)

    def on_proj_matrix_created(self):
        self.p_mat = self.proj_mat_creator.pm
        self.num_proj = len(self.p_mat)
        self.on_num_proj_changed()
        self.proj_mat_creator_widget.setVisible(False)

    def on_tf_num_proj_changed(self):
        self.num_proj = int(self.proj_mat_creator.tf_num_proj.text()) + 1
        self.on_num_proj_changed()

    def on_num_proj_changed(self):
        ##first we need to unregister all motions
        self.on_bt_reset_all_proj()
        ##adjust the length of the time curser
        self.sb_time_cursor.setRange(0, int(self.num_proj) - 1)
        ##set current projection to zero and display it
        self.current_proj = 0
        self.set_text_selected_timepoint_label()
        ##init new motion matrix, with new size
        self.motion_parameters = np.zeros((self.num_proj, 6), dtype=np.float32)
        # mp 0 = t_ax
        # mp 1 = t_cor
        # mp 2 = t_sag
        # mp 3 = R_ax
        # mp 4 = R_cor
        # mp 5 = R_sag
        ## set the slider to zero
        self.set_sliders_via_array(self.motion_parameters[self.current_proj, ...])
        self.set_motion_label_and_btn()
        self.sb_time_cursor.setValue(self.current_proj)

    def on_bt_save(self):
        self.p_mat_to_save_file = \
            QtWidgets.QFileDialog.getSaveFileName(self.centralwidget, "Save file", self.p_mat_dir, )[0]
        if self.p_mat_to_save_file != '':
            matrices = self.p_mat_to_save_file + ".matrices"
            motion = self.p_mat_to_save_file + ".motion"
            print(self.p_mat_to_save_file)
            ### multiply motion
            ### save projection matrices
            ### save motion parameters
            motion_corrupted_pmat, rt_list = help_functions.add_motion_to_pMat(self.p_mat, self.motion_parameters)
            readWriteRaw.write(matrices, motion_corrupted_pmat)
            readWriteRaw.write(motion, np.matrix(self.motion_parameters))
            # readWriteRaw.write()

    def on_bt_save_pm(self):
        self.p_mat_to_save_file = \
            QtWidgets.QFileDialog.getSaveFileName(self.centralwidget, "Save file", self.p_mat_dir, )[0]
        if self.p_mat_to_save_file != '':
            matrices = self.p_mat_to_save_file + ".matrices"
            print(self.p_mat_to_save_file)
            ### multiply motion
            ### save projection matrices
            ### save motion parameters
            motion_corrupted_pmat, rt_list = help_functions.add_motion_to_pMat(self.p_mat, self.motion_parameters)
            readWriteRaw.write(matrices, motion_corrupted_pmat)

    def on_load_motion_file(self):
        motion_fn = QtWidgets.QFileDialog.getOpenFileName(self.centralwidget, 'Load Motion File',
                                                          self.p_mat_dir
                                                          , "Motion files (*.motion)"
                                                          )[
            0]  # getOpenFileName returns tupel with second entry beeing the filter
        motion_list = readWriteRaw.read(motion_fn)
        if (self.num_proj != len(motion_list)):
            print('Dimension of loaded motion_file and loaded projections do not fit')
            print('Length of loaded Projections: {}'.format(self.num_proj))
            print('Length of loaded Motion: {}'.format(len(motion_list)))
        else:
            self.motion_parameters = []
            for motion in motion_list:
                self.motion_parameters.append(motion[0])
            for i in range(0, self.num_proj):
                self.register_projection_n(i)
            self.sb_time_cursor.setValue(1)
            self.sb_time_cursor.setValue(0)

    def on_bt_load_pm(self):
        self.p_mat_file = QtWidgets.QFileDialog.getOpenFileName(self.centralwidget, 'Load Projection Matrices',
                                                                self.p_mat_dir
                                                                #     , "Image files (*.jpg *.gif *matrices)"
                                                                )[
            0]  # getOpenFileName returns tupel with second entry beeing the filter
        self.load_p_mat()

    def load_p_mat(self):
        ##read projection matrix from file
        self.p_mat = readWriteRaw.read(self.p_mat_file)
        self.num_proj = len(self.p_mat)
        ##init projection motion
        self.on_num_proj_changed()


class Window(QtWidgets.QMainWindow):
    resized = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(Window, self).__init__(parent=parent)

    ###We need to map the show event onto the resized signal,
    ### such that the respective MPR are initialized to a good size
    def showEvent(self, event):
        self.resized.emit()
        return super(Window, self).showEvent(event)

    def resizeEvent(self, event):
        self.resized.emit()
        return super(Window, self).resizeEvent(event)


if __name__ == '__main__':
    # if os.name == "nt":  # if windows
    #    import PyQt5
    #   pyqt_plugins = os.path.join(os.path.dirname(PyQt5.__file__),
    # "..", "..", "..", "Library", "plugins")
    #  QtWidgets.QApplication.addLibraryPath(pyqt_plugins)
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = Window()
    prog = MotionCreator_logic(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
