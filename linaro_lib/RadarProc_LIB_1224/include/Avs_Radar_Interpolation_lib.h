#ifndef _AVS_RADAR_INTERPOLATION_LIB_H_
#define _AVS_RADAR_INTERPOLATION_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Avs_Radar.h"
#include "Avs_Radar_lib.h"
#include "Avs_Base.h"
#include "Avs_Common.h"
#include "Avs_Radar_Common_lib.h"
#include <math.h>

void bilinear_interpolation_im(	AVS_RADAR_PREPROCESS_IN *inbuf, 
								AVS_RADAR_FILTER		*avs_filter);

void bilinear_output_info(	AVS_RADAR_FILTER			*avs_filter,
							AVS_RADAR_PREPROCESS_OUT	*outbuf);

#ifdef __cplusplus
}
#endif

#endif