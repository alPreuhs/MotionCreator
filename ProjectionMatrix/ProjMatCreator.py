# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ProjMatCreator.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_projectionMatrixCreator(object):
    def setupUi(self, projectionMatrixCreator):
        projectionMatrixCreator.setObjectName("projectionMatrixCreator")
        projectionMatrixCreator.resize(984, 930)
        self.gridLayout = QtWidgets.QGridLayout(projectionMatrixCreator)
        self.gridLayout.setObjectName("gridLayout")
        self.frame_5 = QtWidgets.QFrame(projectionMatrixCreator)
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame_5)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_17 = QtWidgets.QFrame(self.frame_5)
        self.frame_17.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_17.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_17.setObjectName("frame_17")
        self.gridLayout_14 = QtWidgets.QGridLayout(self.frame_17)
        self.gridLayout_14.setObjectName("gridLayout_14")
        self.sl_num_proj = QtWidgets.QSlider(self.frame_17)
        self.sl_num_proj.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_num_proj.setMinimum(0)
        self.sl_num_proj.setMaximum(2000)
        self.sl_num_proj.setProperty("value", 200)
        self.sl_num_proj.setOrientation(QtCore.Qt.Horizontal)
        self.sl_num_proj.setObjectName("sl_num_proj")
        self.gridLayout_14.addWidget(self.sl_num_proj, 1, 0, 1, 2)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_14.addItem(spacerItem, 0, 2, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.frame_17)
        self.label_12.setObjectName("label_12")
        self.gridLayout_14.addWidget(self.label_12, 0, 0, 1, 1)
        self.tf_num_proj = QtWidgets.QLineEdit(self.frame_17)
        self.tf_num_proj.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_num_proj.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_num_proj.setObjectName("tf_num_proj")
        self.gridLayout_14.addWidget(self.tf_num_proj, 1, 2, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_14.addItem(spacerItem1, 0, 1, 1, 1)
        self.verticalLayout.addWidget(self.frame_17)
        self.frame_24 = QtWidgets.QFrame(self.frame_5)
        self.frame_24.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_24.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_24.setObjectName("frame_24")
        self.gridLayout_21 = QtWidgets.QGridLayout(self.frame_24)
        self.gridLayout_21.setObjectName("gridLayout_21")
        self.sl_proj_sparsity = QtWidgets.QSlider(self.frame_24)
        self.sl_proj_sparsity.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_proj_sparsity.setMinimum(1)
        self.sl_proj_sparsity.setMaximum(200)
        self.sl_proj_sparsity.setProperty("value", 8)
        self.sl_proj_sparsity.setOrientation(QtCore.Qt.Horizontal)
        self.sl_proj_sparsity.setObjectName("sl_proj_sparsity")
        self.gridLayout_21.addWidget(self.sl_proj_sparsity, 1, 0, 1, 2)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_21.addItem(spacerItem2, 0, 2, 1, 1)
        self.label_24 = QtWidgets.QLabel(self.frame_24)
        self.label_24.setObjectName("label_24")
        self.gridLayout_21.addWidget(self.label_24, 0, 0, 1, 1)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_21.addItem(spacerItem3, 0, 1, 1, 1)
        self.tf_proj_sparsity = QtWidgets.QLineEdit(self.frame_24)
        self.tf_proj_sparsity.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_proj_sparsity.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_proj_sparsity.setObjectName("tf_proj_sparsity")
        self.gridLayout_21.addWidget(self.tf_proj_sparsity, 1, 2, 1, 1)
        self.verticalLayout.addWidget(self.frame_24)
        self.frame_8 = QtWidgets.QFrame(self.frame_5)
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.frame_8)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_5 = QtWidgets.QLabel(self.frame_8)
        self.label_5.setObjectName("label_5")
        self.gridLayout_6.addWidget(self.label_5, 0, 0, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_6.addItem(spacerItem4, 0, 3, 1, 1)
        self.tf_start_angle = QtWidgets.QLineEdit(self.frame_8)
        self.tf_start_angle.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_start_angle.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_start_angle.setObjectName("tf_start_angle")
        self.gridLayout_6.addWidget(self.tf_start_angle, 1, 3, 1, 1)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_6.addItem(spacerItem5, 0, 1, 1, 1)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_6.addItem(spacerItem6, 0, 2, 1, 1)
        self.sl_start_angle = QtWidgets.QSlider(self.frame_8)
        self.sl_start_angle.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_start_angle.setMaximum(360)
        self.sl_start_angle.setSingleStep(1)
        self.sl_start_angle.setPageStep(10)
        self.sl_start_angle.setOrientation(QtCore.Qt.Horizontal)
        self.sl_start_angle.setObjectName("sl_start_angle")
        self.gridLayout_6.addWidget(self.sl_start_angle, 1, 0, 1, 3)
        self.verticalLayout.addWidget(self.frame_8)
        self.frame_3 = QtWidgets.QFrame(self.frame_5)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.frame_3)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label = QtWidgets.QLabel(self.frame_3)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 0, 0, 1, 1)
        self.tf_stop_angle = QtWidgets.QLineEdit(self.frame_3)
        self.tf_stop_angle.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_stop_angle.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_stop_angle.setObjectName("tf_stop_angle")
        self.gridLayout_2.addWidget(self.tf_stop_angle, 1, 3, 1, 1)
        spacerItem7 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_2.addItem(spacerItem7, 0, 3, 1, 1)
        spacerItem8 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_2.addItem(spacerItem8, 0, 1, 1, 1)
        spacerItem9 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_2.addItem(spacerItem9, 0, 2, 1, 1)
        self.sl_stop_angle = QtWidgets.QSlider(self.frame_3)
        self.sl_stop_angle.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_stop_angle.setBaseSize(QtCore.QSize(0, 0))
        self.sl_stop_angle.setMaximum(360)
        self.sl_stop_angle.setProperty("value", 100)
        self.sl_stop_angle.setOrientation(QtCore.Qt.Horizontal)
        self.sl_stop_angle.setObjectName("sl_stop_angle")
        self.gridLayout_2.addWidget(self.sl_stop_angle, 1, 0, 1, 3)
        self.verticalLayout.addWidget(self.frame_3)
        self.frame_9 = QtWidgets.QFrame(self.frame_5)
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.frame_9)
        self.gridLayout_7.setObjectName("gridLayout_7")
        spacerItem10 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_7.addItem(spacerItem10, 0, 3, 1, 1)
        self.tf_SID = QtWidgets.QLineEdit(self.frame_9)
        self.tf_SID.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_SID.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_SID.setObjectName("tf_SID")
        self.gridLayout_7.addWidget(self.tf_SID, 1, 3, 1, 1)
        self.sl_SID = QtWidgets.QSlider(self.frame_9)
        self.sl_SID.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_SID.setMinimum(1)
        self.sl_SID.setMaximum(3000)
        self.sl_SID.setProperty("value", 1200)
        self.sl_SID.setOrientation(QtCore.Qt.Horizontal)
        self.sl_SID.setObjectName("sl_SID")
        self.gridLayout_7.addWidget(self.sl_SID, 1, 0, 1, 3)
        self.label_6 = QtWidgets.QLabel(self.frame_9)
        self.label_6.setObjectName("label_6")
        self.gridLayout_7.addWidget(self.label_6, 0, 0, 1, 1)
        spacerItem11 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_7.addItem(spacerItem11, 0, 1, 1, 2)
        self.verticalLayout.addWidget(self.frame_9)
        self.frame_10 = QtWidgets.QFrame(self.frame_5)
        self.frame_10.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_10.setObjectName("frame_10")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.frame_10)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.sl_SISOD = QtWidgets.QSlider(self.frame_10)
        self.sl_SISOD.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_SISOD.setMinimum(1)
        self.sl_SISOD.setMaximum(2000)
        self.sl_SISOD.setSliderPosition(750)
        self.sl_SISOD.setOrientation(QtCore.Qt.Horizontal)
        self.sl_SISOD.setObjectName("sl_SISOD")
        self.gridLayout_8.addWidget(self.sl_SISOD, 1, 0, 1, 3)
        spacerItem12 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_8.addItem(spacerItem12, 0, 3, 1, 1)
        self.tf_SISOD = QtWidgets.QLineEdit(self.frame_10)
        self.tf_SISOD.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_SISOD.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_SISOD.setObjectName("tf_SISOD")
        self.gridLayout_8.addWidget(self.tf_SISOD, 1, 3, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.frame_10)
        self.label_7.setObjectName("label_7")
        self.gridLayout_8.addWidget(self.label_7, 0, 0, 1, 1)
        spacerItem13 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_8.addItem(spacerItem13, 0, 1, 1, 2)
        self.verticalLayout.addWidget(self.frame_10)
        self.frame_7 = QtWidgets.QFrame(self.frame_5)
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.frame_7)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.sl_size_U = QtWidgets.QSlider(self.frame_7)
        self.sl_size_U.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_size_U.setMinimum(1)
        self.sl_size_U.setMaximum(3000)
        self.sl_size_U.setProperty("value", 1240)
        self.sl_size_U.setSliderPosition(1240)
        self.sl_size_U.setOrientation(QtCore.Qt.Horizontal)
        self.sl_size_U.setObjectName("sl_size_U")
        self.gridLayout_5.addWidget(self.sl_size_U, 1, 0, 1, 3)
        spacerItem14 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_5.addItem(spacerItem14, 0, 3, 1, 1)
        self.tf_size_U = QtWidgets.QLineEdit(self.frame_7)
        self.tf_size_U.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_size_U.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_size_U.setObjectName("tf_size_U")
        self.gridLayout_5.addWidget(self.tf_size_U, 1, 3, 1, 1)
        spacerItem15 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_5.addItem(spacerItem15, 0, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.frame_7)
        self.label_4.setObjectName("label_4")
        self.gridLayout_5.addWidget(self.label_4, 0, 0, 1, 1)
        spacerItem16 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_5.addItem(spacerItem16, 0, 2, 1, 1)
        self.verticalLayout.addWidget(self.frame_7)
        self.frame_15 = QtWidgets.QFrame(self.frame_5)
        self.frame_15.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_15.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_15.setObjectName("frame_15")
        self.gridLayout_12 = QtWidgets.QGridLayout(self.frame_15)
        self.gridLayout_12.setObjectName("gridLayout_12")
        self.sl_size_V = QtWidgets.QSlider(self.frame_15)
        self.sl_size_V.setMinimumSize(QtCore.QSize(50, 0))
        self.sl_size_V.setMinimum(1)
        self.sl_size_V.setMaximum(3000)
        self.sl_size_V.setProperty("value", 960)
        self.sl_size_V.setOrientation(QtCore.Qt.Horizontal)
        self.sl_size_V.setObjectName("sl_size_V")
        self.gridLayout_12.addWidget(self.sl_size_V, 1, 0, 1, 3)
        spacerItem17 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_12.addItem(spacerItem17, 0, 3, 1, 1)
        self.tf_size_V = QtWidgets.QLineEdit(self.frame_15)
        self.tf_size_V.setMinimumSize(QtCore.QSize(40, 0))
        self.tf_size_V.setMaximumSize(QtCore.QSize(40, 16777215))
        self.tf_size_V.setObjectName("tf_size_V")
        self.gridLayout_12.addWidget(self.tf_size_V, 1, 3, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.frame_15)
        self.label_10.setObjectName("label_10")
        self.gridLayout_12.addWidget(self.label_10, 0, 0, 1, 1)
        spacerItem18 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_12.addItem(spacerItem18, 0, 2, 1, 1)
        spacerItem19 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_12.addItem(spacerItem19, 0, 1, 1, 1)
        self.verticalLayout.addWidget(self.frame_15)
        self.frame_21 = QtWidgets.QFrame(self.frame_5)
        self.frame_21.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_21.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_21.setObjectName("frame_21")
        self.gridLayout_18 = QtWidgets.QGridLayout(self.frame_21)
        self.gridLayout_18.setObjectName("gridLayout_18")
        self.tf_px_U = QtWidgets.QLineEdit(self.frame_21)
        self.tf_px_U.setMinimumSize(QtCore.QSize(20, 0))
        self.tf_px_U.setObjectName("tf_px_U")
        self.gridLayout_18.addWidget(self.tf_px_U, 0, 1, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.frame_21)
        self.label_16.setObjectName("label_16")
        self.gridLayout_18.addWidget(self.label_16, 0, 0, 1, 1)
        self.label_25 = QtWidgets.QLabel(self.frame_21)
        self.label_25.setObjectName("label_25")
        self.gridLayout_18.addWidget(self.label_25, 1, 0, 1, 1)
        self.tf_px_V = QtWidgets.QLineEdit(self.frame_21)
        self.tf_px_V.setMinimumSize(QtCore.QSize(20, 0))
        self.tf_px_V.setObjectName("tf_px_V")
        self.gridLayout_18.addWidget(self.tf_px_V, 1, 1, 1, 1)
        self.frame = QtWidgets.QFrame(self.frame_21)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.gridLayout_18.addWidget(self.frame, 2, 0, 1, 2)
        self.verticalLayout.addWidget(self.frame_21)
        self.frame_3.raise_()
        self.frame_7.raise_()
        self.frame_10.raise_()
        self.frame_15.raise_()
        self.frame_17.raise_()
        self.frame_21.raise_()
        self.frame_9.raise_()
        self.frame_24.raise_()
        self.frame_8.raise_()
        self.gridLayout.addWidget(self.frame_5, 0, 0, 1, 1)
        self.View3D = QtWidgets.QFrame(projectionMatrixCreator)
        self.View3D.setMinimumSize(QtCore.QSize(685, 200))
        self.View3D.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.View3D.setFrameShadow(QtWidgets.QFrame.Raised)
        self.View3D.setObjectName("View3D")
        self.gridLayout.addWidget(self.View3D, 0, 1, 2, 1)
        self.bt_reset = QtWidgets.QPushButton(projectionMatrixCreator)
        self.bt_reset.setObjectName("bt_reset")
        self.gridLayout.addWidget(self.bt_reset, 1, 0, 1, 1)
        self.bt_confirm_proj_mat = QtWidgets.QPushButton(projectionMatrixCreator)
        self.bt_confirm_proj_mat.setObjectName("bt_confirm_proj_mat")
        self.gridLayout.addWidget(self.bt_confirm_proj_mat, 2, 0, 1, 1)

        self.retranslateUi(projectionMatrixCreator)
        QtCore.QMetaObject.connectSlotsByName(projectionMatrixCreator)

    def retranslateUi(self, projectionMatrixCreator):
        _translate = QtCore.QCoreApplication.translate
        projectionMatrixCreator.setWindowTitle(_translate("projectionMatrixCreator", "Form"))
        self.label_12.setText(_translate("projectionMatrixCreator", "Set Number of Projections"))
        self.tf_num_proj.setText(_translate("projectionMatrixCreator", "200"))
        self.label_24.setText(_translate("projectionMatrixCreator", "Set Visualization Sparsity"))
        self.tf_proj_sparsity.setText(_translate("projectionMatrixCreator", "8"))
        self.label_5.setText(_translate("projectionMatrixCreator", "Start Angle"))
        self.tf_start_angle.setText(_translate("projectionMatrixCreator", "0"))
        self.label.setText(_translate("projectionMatrixCreator", "Stop Angle"))
        self.tf_stop_angle.setText(_translate("projectionMatrixCreator", "100"))
        self.tf_SID.setText(_translate("projectionMatrixCreator", "1200"))
        self.label_6.setText(_translate("projectionMatrixCreator", "Source Detector Distance"))
        self.tf_SISOD.setText(_translate("projectionMatrixCreator", "750"))
        self.label_7.setText(_translate("projectionMatrixCreator", "Source Isocenter Distance"))
        self.tf_size_U.setText(_translate("projectionMatrixCreator", "1240"))
        self.label_4.setText(_translate("projectionMatrixCreator", "Detector u size"))
        self.tf_size_V.setText(_translate("projectionMatrixCreator", "960"))
        self.label_10.setText(_translate("projectionMatrixCreator", "Detector v size"))
        self.tf_px_U.setText(_translate("projectionMatrixCreator", "0.305"))
        self.label_16.setText(_translate("projectionMatrixCreator", "Pixel Spacing U"))
        self.label_25.setText(_translate("projectionMatrixCreator", "Pixel Spacing V"))
        self.tf_px_V.setText(_translate("projectionMatrixCreator", "0.305"))
        self.bt_reset.setText(_translate("projectionMatrixCreator", "Reset all Values"))
        self.bt_confirm_proj_mat.setText(_translate("projectionMatrixCreator", "Confirm "))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    projectionMatrixCreator = QtWidgets.QWidget()
    ui = Ui_projectionMatrixCreator()
    ui.setupUi(projectionMatrixCreator)
    projectionMatrixCreator.show()
    sys.exit(app.exec_())

