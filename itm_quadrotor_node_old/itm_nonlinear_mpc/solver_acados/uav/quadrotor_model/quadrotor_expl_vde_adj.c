/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) quadrotor_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s3[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

/* quadrotor_expl_vde_adj:(i0[9],i1[9],i2[3],i3[4])->(o0[12]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a3, a4, a5, a6, a7, a8, a9;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[1]? arg[1][1] : 0;
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[1]? arg[1][2] : 0;
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[0]? arg[0][6] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][8] : 0;
  a3=sin(a2);
  a4=arg[2]? arg[2][2] : 0;
  a5=arg[1]? arg[1][3] : 0;
  a6=(a4*a5);
  a7=(a3*a6);
  a1=(a1*a7);
  a7=arg[1]? arg[1][6] : 0;
  a8=arg[3]? arg[3][1] : 0;
  a7=(a7/a8);
  a8=sin(a0);
  a9=arg[0]? arg[0][7] : 0;
  a10=cos(a9);
  a11=arg[1]? arg[1][5] : 0;
  a12=(a4*a11);
  a13=(a10*a12);
  a8=(a8*a13);
  a8=(a7+a8);
  a13=cos(a0);
  a14=cos(a2);
  a15=arg[1]? arg[1][4] : 0;
  a4=(a4*a15);
  a16=(a14*a4);
  a13=(a13*a16);
  a8=(a8+a13);
  a13=sin(a0);
  a16=sin(a9);
  a17=sin(a2);
  a18=(a17*a4);
  a19=(a16*a18);
  a13=(a13*a19);
  a8=(a8+a13);
  a1=(a1-a8);
  a8=sin(a0);
  a13=cos(a2);
  a19=sin(a9);
  a20=(a19*a6);
  a21=(a13*a20);
  a8=(a8*a21);
  a1=(a1-a8);
  if (res[0]!=0) res[0][6]=a1;
  a1=cos(a9);
  a8=cos(a0);
  a18=(a8*a18);
  a1=(a1*a18);
  a18=arg[1]? arg[1][7] : 0;
  a21=arg[3]? arg[3][3] : 0;
  a18=(a18/a21);
  a21=sin(a9);
  a22=cos(a0);
  a12=(a22*a12);
  a21=(a21*a12);
  a21=(a18+a21);
  a1=(a1-a21);
  a9=cos(a9);
  a21=cos(a0);
  a13=(a21*a13);
  a12=(a13*a6);
  a9=(a9*a12);
  a1=(a1+a9);
  if (res[0]!=0) res[0][7]=a1;
  a1=sin(a2);
  a9=sin(a0);
  a12=(a9*a4);
  a1=(a1*a12);
  a12=cos(a2);
  a8=(a8*a16);
  a4=(a8*a4);
  a12=(a12*a4);
  a1=(a1+a12);
  a12=cos(a2);
  a0=sin(a0);
  a6=(a0*a6);
  a12=(a12*a6);
  a1=(a1+a12);
  a2=sin(a2);
  a21=(a21*a20);
  a2=(a2*a21);
  a1=(a1-a2);
  if (res[0]!=0) res[0][8]=a1;
  a1=arg[3]? arg[3][0] : 0;
  a1=(a1*a7);
  if (res[0]!=0) res[0][9]=a1;
  a1=arg[3]? arg[3][2] : 0;
  a1=(a1*a18);
  if (res[0]!=0) res[0][10]=a1;
  a10=(a10*a22);
  a10=(a10*a11);
  a8=(a8*a17);
  a14=(a14*a9);
  a8=(a8-a14);
  a8=(a8*a15);
  a10=(a10+a8);
  a13=(a13*a19);
  a0=(a0*a3);
  a13=(a13+a0);
  a13=(a13*a5);
  a10=(a10+a13);
  if (res[0]!=0) res[0][11]=a10;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_expl_vde_adj_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_expl_vde_adj_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_expl_vde_adj_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
