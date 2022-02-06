//
// MATLAB Compiler: 8.3 (R2021b)
// Date: Thu Feb  3 15:49:00 2022
// Arguments:
// "-B""macro_default""-W""cpplib:libsvd_mean_recompute_denoise_first_eigen_zero
// ,all""-T""link:lib""-d""/home/jjj/NGCLab/ThermalOdo/img/libsvd_mean_recompute
// _denoise_first_eigen_zero/for_testing""-v""/home/jjj/NGCLab/ThermalOdo/img/sv
// d_mean_recompute_denoise_first_eigen_zero.m"
//

#define EXPORTING_libsvd_mean_recompute_denoise_first_eigen_zero 1
#include "libsvd_mean_recompute_denoise_first_eigen_zero.h"

static HMCRINSTANCE _mcr_inst = NULL; /* don't use nullptr; this may be either C or C++ */

#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultPrintHandler(const char *s)
{
    return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern C block */
#endif

#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

static int mclDefaultErrorHandler(const char *s)
{
    int written = 0;
    size_t len = 0;
    len = strlen(s);
    written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
    if (len > 0 && s[ len-1 ] != '\n')
        written += mclWrite(2 /* stderr */, "\n", sizeof(char));
    return written;
}

#ifdef __cplusplus
} /* End extern C block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API
#define LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API /* No special import/export declaration */
#endif

LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
bool MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
    if (_mcr_inst)
        return true;
    if (!mclmcrInitialize())
        return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream((void 
                                                     *)(libsvd_mean_recompute_denoise_first_eigen_zeroInitializeWithHandlers));
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(&_mcr_inst,
                                                             error_handler, 
                                                             print_handler,
                                                             ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
    return true;
}

LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
bool MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroInitialize(void)
{
    return 
                                                                              libsvd_mean_recompute_denoise_first_eigen_zeroInitializeWithHandlers(mclDefaultErrorHandler, 
                                                                              mclDefaultPrintHandler);
}

LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
void MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroTerminate(void)
{
    if (_mcr_inst)
        mclTerminateInstance(&_mcr_inst);
}

LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
void MW_CALL_CONV libsvd_mean_recompute_denoise_first_eigen_zeroPrintStackTrace(void) 
{
    char** stackTrace;
    int stackDepth = mclGetStackTrace(&stackTrace);
    int i;
    for(i=0; i<stackDepth; i++)
    {
        mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
        mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
    }
    mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_libsvd_mean_recompute_denoise_first_eigen_zero_C_API 
bool MW_CALL_CONV mlxSvd_mean_recompute_denoise_first_eigen_zero(int nlhs, mxArray 
                                                                 *plhs[], int nrhs, 
                                                                 mxArray *prhs[])
{
    return mclFeval(_mcr_inst, "svd_mean_recompute_denoise_first_eigen_zero", nlhs, plhs, 
                  nrhs, prhs);
}

LIB_libsvd_mean_recompute_denoise_first_eigen_zero_CPP_API 
void MW_CALL_CONV svd_mean_recompute_denoise_first_eigen_zero(int nargout, mwArray& m, 
                                                              const mwArray& img)
{
    mclcppMlfFeval(_mcr_inst, "svd_mean_recompute_denoise_first_eigen_zero", nargout, 1, 1, &m, &img);
}

