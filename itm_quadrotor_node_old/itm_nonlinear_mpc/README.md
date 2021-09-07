# itm_nonlinear_mpc

## ACADO solvers

### CPP version

The CPP version is stored in the folder *solver_acado_resource*. Basically, one should install the ACADO library locally according to the introduction from the [official website](https://acado.github.io). After compiling the code, an environment setup script is generated with the name *acado_env.sh* as default in the compile folder. If one wants to inform the system the setting of the ACADO,

```bash
source acado_env.sh
```

before the local algorithm compile.

In the folder *solver_acado_resource_ethz*, the code is executed

```bash
cmake . && make
```

> Note that, for the macOS users, the clang will be the default c/c++ compiler, which are not working with ACADO. Therefore, one should use

```bash
    cmake -DCMAKE_C_COMPILER=/usr/local/Cellar/gcc/10.2.0/bin/gcc-10 -DCMAKE_CXX_COMPILER=/usr/local/Cellar/gcc/10.2.0/bin/g++-10 ..
```

The generated code is stored in *solver* folder.

Then, go to the *solver_ethz* folder, and execute the generated program in the last step. It will create pre-coded file automatically. Since the ACADO depends on qpoases to solve the linear problem, one needs to copy the qpoases files from the ACADO source folder.

### Python version

For the Python ACADO, one needs to create/modify two setup file for C and Python respectively, **acado_uav.c** and **setup.py**. One can utilize

```bash
python setup.py install --user
```

to the compile and install the library for the Python.

## Launch modules

### ITM_PID_SIM_OFFBOARD

Using the default PID controller from PX4 to control the quadrotor.

### ITM
