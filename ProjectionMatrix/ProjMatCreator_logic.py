import sys

sys.path.append("..")
from Math import projection
from ProjectionMatrix.vtk_window_pm import vtk_window_pm
import numpy as np
from ProjectionMatrix.vtk_proj_matrix import vtk_proj_matrix
from PyQt5 import QtWidgets

from ProjectionMatrix.ProjMatCreator import Ui_projectionMatrixCreator


class ProjMatCreator_logic(Ui_projectionMatrixCreator):
    def __init__(self, widget):
        Ui_projectionMatrixCreator.__init__(self)
        ##set up the UI
        self.setupUi(widget)
        self.init_variables()
        self.connect_sliders()
        self.connect_textfields()
        self.connect_buttons()
        self.vtk_handle = vtk_window_pm()
        self.vtk_handle.vtkWidget(self.View3D)
        self.create_tajectory()

    def connect_buttons(self):
        self.bt_reset.clicked.connect(self.on_reset_all_values)

    def init_variables(self):
        self.initial_values = [200, 0, 100, 1200, 750, 1240, 960, 0.305, 0.305, 8, 0, 0, 1]
        self.current_values = self.initial_values.copy()
        self.pm_actor_list = []
        self.pm = []
        self.last_density = self.current_values[9]

    def on_update_matrices(self):
        self.pm_actor_list = []
        self.pm = []

        start = self.current_values[1]
        stop = self.current_values[2]
        num_proj = max(self.current_values[0], 1)
        delta_ang = (stop - start) / num_proj
        axis = np.matrix([self.current_values[-3], self.current_values[-2], self.current_values[-1]])

        offset_u = self.current_values[5] / 2
        offset_v = self.current_values[6] / 2

        sid = self.current_values[3]
        sisod = self.current_values[4]
        px_sp = self.current_values[7]
        default_pm = projection.create_default_projection_matrix(pixel_spacing=px_sp,
                                                                 sid=sid, sisod=sisod, offset_u=offset_u,
                                                                 offset_v=offset_v)

        for i in range(int(self.current_values[0])):
            rot = projection.get_rotation_matrix_by_axis_and_angle(axis.T, start + delta_ang * i,
                                                                   make_matrix_homogen=True)
            pm = default_pm * rot
            self.pm.append(pm)
            self.pm_actor_list.append(vtk_proj_matrix(pm, sid, offset_u * 2, offset_v * 2))

    def connect_sliders(self):
        ##connect sliders
        self.sl_num_proj.valueChanged.connect(lambda x: self.tf_num_proj.setText(str(x)))
        self.sl_proj_sparsity.valueChanged.connect(lambda x: self.tf_proj_sparsity.setText(str(x)))
        self.sl_start_angle.valueChanged.connect(lambda x: self.tf_start_angle.setText(str(x)))
        self.sl_stop_angle.valueChanged.connect(lambda x: self.tf_stop_angle.setText(str(x)))
        self.sl_SID.valueChanged.connect(lambda x: self.tf_SID.setText(str(x)))
        self.sl_SISOD.valueChanged.connect(lambda x: self.tf_SISOD.setText(str(x)))
        self.sl_size_U.valueChanged.connect(lambda x: self.tf_size_U.setText(str(x)))
        self.sl_size_V.valueChanged.connect(lambda x: self.tf_size_V.setText(str(x)))

    def connect_textfields(self):
        self.tf_num_proj.textChanged.connect(lambda x: self.on_proj_para_changed(x, 0))
        self.tf_proj_sparsity.textChanged.connect(lambda x: self.on_proj_para_changed(x, 9))
        self.tf_start_angle.textChanged.connect(lambda x: self.on_proj_para_changed(x, 1))
        self.tf_stop_angle.textChanged.connect(lambda x: self.on_proj_para_changed(x, 2))
        self.tf_SID.textChanged.connect(lambda x: self.on_proj_para_changed(x, 3))
        self.tf_SISOD.textChanged.connect(lambda x: self.on_proj_para_changed(x, 4))
        self.tf_size_U.textChanged.connect(lambda x: self.on_proj_para_changed(x, 5))
        self.tf_size_V.textChanged.connect(lambda x: self.on_proj_para_changed(x, 6))
        self.tf_px_U.textChanged.connect(lambda x: self.on_proj_para_changed(x, 7))
        self.tf_px_V.textChanged.connect(lambda x: self.on_proj_para_changed(x, 8))

        ###sync textbox with sliders
        self.tf_num_proj.textChanged.connect(lambda x: self.sl_num_proj.setSliderPosition(int(x)))
        self.tf_proj_sparsity.textChanged.connect(lambda x: self.sl_proj_sparsity.setSliderPosition(int(x)))
        self.tf_start_angle.textChanged.connect(lambda x: self.sl_start_angle.setSliderPosition(int(x)))
        self.tf_stop_angle.textChanged.connect(lambda x: self.sl_stop_angle.setSliderPosition(int(x)))
        self.tf_SID.textChanged.connect(lambda x: self.sl_SID.setSliderPosition(int(x)))
        self.tf_SISOD.textChanged.connect(lambda x: self.sl_SISOD.setSliderPosition(int(x)))
        self.tf_size_U.textChanged.connect(lambda x: self.sl_size_U.setSliderPosition(int(x)))
        self.tf_size_V.textChanged.connect(lambda x: self.sl_size_V.setSliderPosition(int(x)))

    def disable_slider_connections(self, boolean):
        self.sl_num_proj.blockSignals(boolean)
        self.sl_start_angle.blockSignals(boolean)
        self.sl_stop_angle.blockSignals(boolean)
        self.sl_SID.blockSignals(boolean)
        self.sl_SISOD.blockSignals(boolean)
        self.sl_size_U.blockSignals(boolean)
        self.sl_size_V.blockSignals(boolean)

    def on_proj_para_changed(self, value, identifier):
        cur_para = self.current_values
        create_trajectory = self.create_tajectory
        cur_para[identifier] = float(value)
        create_trajectory()

    def on_set_projection_values(self, value, identifier):
            if identifier == 0:
                self.tf_num_proj.setText(str(value))
                self.sl_num_proj.setSliderPosition(int(value))
            elif identifier == 1:
                self.tf_start_angle.setText(str(value))
                self.sl_start_angle.setSliderPosition(int(value))
            elif identifier == 2:
                self.tf_stop_angle.setText(str(value))
                self.sl_stop_angle.setSliderPosition(int(value))
            elif identifier == 3:
                self.tf_SID.setText(str(value))
                self.sl_SID.setSliderPosition(int(value))
            elif identifier == 4:
                self.tf_SISOD.setText(str(value))
                self.sl_SISOD.setSliderPosition(int(value))
            elif identifier == 5:
                self.tf_size_U.setText(str(value))
                self.sl_size_U.setSliderPosition(int(value))
            elif identifier == 6:
                self.tf_size_V.setText(str(value))
                self.sl_size_V.setSliderPosition(int(value))
            elif identifier == 7:
                self.tf_px_U.setText(str(value))
            elif identifier == 8:
                self.tf_px_V.setText(str(value))

    def create_tajectory(self):
        if len(self.pm_actor_list) > 1:
            tmp_list = self.pm_actor_list[::int(self.last_density)]
            for act in self.pm_actor_list:
                self.vtk_handle.remove_actor(act)
        self.on_update_matrices()
        self.last_density = self.current_values[9]
        tmp_list = self.pm_actor_list[::int(self.last_density)]
        for actor in tmp_list:
            self.vtk_handle.add_actors(actor)
        self.vtk_handle.update()

    def on_reset_all_values(self):
        for i in range(len(self.initial_values)):
            self.on_set_projection_values(self.initial_values[i], i)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QDialog()
    prog = ProjMatCreator_logic(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())