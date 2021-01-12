#ifndef _AVS_RADAR_COMMON_LIB_H_
#define _AVS_RADAR_COMMON_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Avs_Base.h"
#include "Avs_Common.h"

#define AVS_MAX(a, b) ((a) > (b) ? (a) : (b))
#define AVS_MIN(a, b) ((a) < (b) ? (a) : (b))

typedef struct _AVS_BUF_ {
	void *start;
	void *end;
	void *cur_pos;
}AVS_BUF;

void *AVS_COM_alloc_buffer(	AVS_BUF *avs_buf, 
							int size);

float get_scale_value(	float	*data, 
						float	scale_i, 
						float	scale_j, 
						int		width, 
						int		height);

#ifdef __cplusplus
}
#endif

#endif
