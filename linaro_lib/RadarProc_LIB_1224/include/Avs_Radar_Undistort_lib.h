#ifndef _AVS_RADAR_UNDISTORT_LIB_H_
#define _AVS_RADAR_UNDISTORT_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Avs_Radar.h"
#include "Avs_Radar_lib.h"
#include "Avs_Base.h"
#include "Avs_Common.h"
#include "Avs_Radar_Common_lib.h"
#include <math.h>

void undistort_im_nearest(	AVS_RADAR_PREPROCESS_IN *inbuf,
							AVS_RADAR_FILTER *avs_filter);

void undistort_im_bi(	AVS_RADAR_PREPROCESS_IN *inbuf, 
						AVS_RADAR_FILTER *avs_filter);

void undistort_output_info(	AVS_RADAR_PREPROCESS_IN		*inbuf,
							AVS_RADAR_FILTER			*avs_filter,
							AVS_RADAR_PREPROCESS_OUT	*outbuf);
#ifdef __cplusplus
}
#endif

#endif