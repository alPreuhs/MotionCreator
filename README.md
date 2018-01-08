# MotionCreator
Python GUI to create rigid motion for any trajectory that is described by several 3x4 projection matrices (e.g. a C-arm short scan)

The prototype is based on Anaconda with python 3.6.1 available under https://www.anaconda.com/download/ and a vtk library which must be installed.
The vtk-whl file (for windows) is located within the prototype under //dependency//vtkWHL//VTK-7.1.1-cp36-cp36m-win_amd64.whl
Thus, for using the prototpye:
  1) the respective anaconda installation must be available 
  2) the vtk whl file must be installed

Thereafter the prototype can be used, eg by using executing the "start_prototype.bat".

Datatypes:
The motion creator supports the reading and writing of raw-projection matrices, i.e. a file consisting of a raw 6 byte header (type: int16)
containing the height and width of the projection matrices and the number of matrices. Thereafter the entries are encoded in float64 (6byte)
format. The encoding is little endian.
