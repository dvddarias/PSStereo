#include "Driver\OniDriverAPI.h"
#include "pseye.h";
#include "XnLib.h"
#include "XnHash.h"
#include "XnEvent.h"
#include "XnPlatform.h"
#include "XnMath.h"
#include "PS1080.h"
#include "D2S.h.h"
#include "S2D.h.h"

#include "new_stereo.h"
#include "global_params.h"

#include "gpu_BM_stereo.h"
#include "gpu_BP_stereo.h"
#include "gpu_CSBP_stereo.h"

#include "cpu_BM_stereo.h"
#include "cpu_SGBM_stereo.h"
#include "cpu_VAR_stereo.h"

using namespace oni::driver;