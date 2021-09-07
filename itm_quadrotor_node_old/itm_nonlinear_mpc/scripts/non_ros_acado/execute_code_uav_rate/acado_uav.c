#define PY_SSIZE_T_CLEAN
#define NPY_NP_DEPRECATED_API NPY_1_18_API_VERSION

#include <Python.h>
// #include <stdio.h>
#include <numpy/arrayobject.h>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#if PY_MAJOR_VERSION >= 3
#define PY3K
#endif


#define NX          ACADO_NX /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */
#define QP_NV       ACADO_QP_NV /* Number of optimization variables. */
#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */
#define N           ACADO_N   /* Number of intervals in the horizon. */

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

static int initialized = 0;
static int verbose = 0;
static int iter = 0;

int getMatrix(PyArrayObject *src, real_t* dest, int rows, int cols, char *desc){
  if ((PyArray_DIM(src, 0) != rows) || (PyArray_DIM(src, 1) != cols))
  {
    PyErr_Format(PyExc_ValueError, "Expecting double matrix %dx%d for %s", rows, cols, desc);
    return 0;
  }
  if (verbose == 2) PySys_WriteStdout("%s:\n", desc);
  for (int i=0; i < rows; i++)
  {
    for (int j=0; j < cols; j++){
      double elem=*((double *)PyArray_GETPTR2(src,i,j));
      dest[i * cols + j] = elem;
      if (verbose == 2) PySys_WriteStdout("%.3f ", elem);
    }
    if (verbose == 2) PySys_WriteStdout("\n");
  }
  if (verbose == 2) PySys_WriteStdout("\n");
  return 1;
};

void setMatrix(real_t *src, PyArrayObject **dest, int rows, int cols){
  npy_intp dims[] = {rows,cols};
  *dest = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
  //*dest = (PyArrayObject *)PyArray_FromDims(2, dims, NPY_DOUBLE);
  double *p = (double*)PyArray_DATA(*dest);
  for (int i=0; i < rows; i++)
  {
    for (int j=0; j < cols; j++){
      p[i * cols + j] = src[i * cols + j];
    }
  }
}


// initializes the boundary of control inputs
static PyObject * mpc_control_bounds(PyObject *self, PyObject *args){
  PyArrayObject *lbValues_;
  PyArrayObject *ubValues_;
  if (!PyArg_ParseTuple(args, "OO", &lbValues_, &ubValues_))
  {
    return NULL;
  }
  if (!getMatrix(lbValues_, acadoVariables.lbValues, 1, QP_NV, "lbw")) return NULL;
  if (!getMatrix(ubValues_, acadoVariables.ubValues, 1, QP_NV, "ubw")) return NULL;
  uint res_;
  res_ = 1;
  PyObject *ret = Py_BuildValue("i", res_);
  return ret;
}

static PyObject* mpc_init(PyObject *self, PyObject *args)
{
  PyArrayObject *x0;
  PyArrayObject *x;
  PyArrayObject *u;
  PyArrayObject *y;
  PyArrayObject *yN;
  PyArrayObject *W;
  PyArrayObject *WN;
  PyArrayObject *od;

  if (!PyArg_ParseTuple(args, "OOOOOOOO", &x0, &x, &u, &y, &yN, &W, &WN, &od)){
      return NULL;
  }

  // get all initial values for vectors and matrices
  if (!getMatrix(x0, acadoVariables.x0, 1,   NX, "x0")) return NULL;
  if (!getMatrix(x, acadoVariables.x,   N+1, NX, "X")) return NULL;
  if (!getMatrix(u, acadoVariables.u,   N,   NU, "U")) return NULL;
  if (!getMatrix(y, acadoVariables.y,   N,   NY, "Y")) return NULL;
  if (!getMatrix(yN, acadoVariables.yN, 1,   NYN,"yN")) return NULL;
  if (!getMatrix(W, acadoVariables.W,   NY, NY, "W")) return NULL;
  if (!getMatrix(WN, acadoVariables.WN, NYN, NYN,"WN")) return NULL;
  if (!getMatrix(od, acadoVariables.od, N+1, NOD, "od")) return NULL;

  // init acado
  acado_initializeSolver();
  // prepare for the loop
  acado_preparationStep();
  // return 1 to indicate that the initialization is accomplished
  uint res_;
  res_ = 1;
  PyObject *ret = Py_BuildValue("i", res_);
  return ret;
}

static PyObject* mpc(PyObject *self, PyObject *args)
{
  PyArrayObject *x0;
  PyArrayObject *u;
  PyArrayObject *x;
  PyArrayObject *y;
  PyArrayObject *yN;
  PyArrayObject *od;

  if (!PyArg_ParseTuple(args, "OOOO", &x0, &y, &yN, &od)){
      return NULL;
  }
  if (!getMatrix(x0, acadoVariables.x0, 1,   NX, "x0")) return NULL;
  if (!getMatrix(y, acadoVariables.y,   N,   NY, "Y")) return NULL;
  if (!getMatrix(yN, acadoVariables.yN, 1,   NYN,"yN")) return NULL;
  if (!getMatrix(od, acadoVariables.od, N+1, NOD, "od")) return NULL;

  acado_feedbackStep();
  iter++;
  setMatrix(acadoVariables.u, &u, N,   NU);
  setMatrix(acadoVariables.x, &x, N+1, NX);
  PyObject* ret = Py_BuildValue("OO", x, u);
  Py_XDECREF(u);
  Py_XDECREF(x);

  acado_shiftStates(1, 0, 0);
  acado_shiftControls( 0 );
  acado_preparationStep();
  return ret;
}



PyObject* check_data(PyObject* self, PyObject* args)
{
  // int iter_;
  // iter_ = iter;
  PyArrayObject *x;
  setMatrix(acadoVariables.W, &x, NY, NY);
  PyObject *ret = Py_BuildValue("O", x);
  Py_XDECREF(x);
  return ret;
}
// registration table
static PyMethodDef acado_uav_methods[]={
    {"mpc", mpc, METH_VARARGS, "func doc"},
    {"check_d", check_data, METH_VARARGS, "func doc"},
    {"mpc_init_function", mpc_init, METH_VARARGS, "func doc"},
    {"mpc_control_bounds", mpc_control_bounds, METH_VARARGS, "func doc"},
    {NULL, NULL, 0, NULL}
};

#ifdef PY3K
// ----------- module definition structure for python3 --------------
static struct PyModuleDef acado_module = {
    PyModuleDef_HEAD_INIT,
    "acado_uav",
    "mod doc",
    -1,
    acado_uav_methods
};
// module initializer for python3
PyMODINIT_FUNC PyInit_acado_uav_rate()
{
  import_array();
  return PyModule_Create(&acado_module);
};
#else
// ---------- module initializer for python2 --------------------------
PyMODINIT_FUNC initacado_uav_rate() {
    Py_InitModule3("acado_uav_rate", acado_uav_methods, "mod doc");
    import_array();
}
#endif
