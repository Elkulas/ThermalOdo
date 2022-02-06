//
// MATLAB Compiler: 8.3 (R2021b)
// Date: Thu Feb  3 15:55:03 2022
// Arguments:
// "-B""macro_default""-W""cpplib:libsvd_mean_recompute_denoise_first_eigen_zero
// ""-T""link:lib""svd_mean_recompute_denoise_first_eigen_zero.m"
//

#ifndef libsvd_mean_recompute_denoise_first_eigen_zero_h
#define libsvd_mean_recompute_denoise_first_eigen_zero_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
#define LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
bool MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
bool MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroInitialize(void);

extern LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
void MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroTerminate(void);

extern LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
void MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
bool MW_CALL_CONV mlxSvd_mean_recompute_denoise_first_eigen_zero(int nlhs, mxArray 
                                                                 *plhs[], int nrhs, 
                                                                 mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_libsvd_mean_recompute_denoise_first_eigen_zero
#define PUBLIC_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API __declspec(dllimport)
#endif

#define LIB_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API PUBLIC_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API

#else

#if !defined(LIB_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API)
#if defined(LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API)
#define LIB_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API
#else
#define LIB_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API void MW_CALL_CONV svd_mean_recompute_denoise_first_eigen_zero(int nargout, mwArray& m, const mwArray& img);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
