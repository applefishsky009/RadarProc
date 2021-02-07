#ifndef _AVS_CAFFE_COMMON_LIB_H_
#define _AVS_CAFFE_COMMON_LIB_H_

#include "Avs_Base.h"
#include "Avs_Common.h"

typedef struct _AVS_CAFFE_BUF_ {
	void *start;
	void *end;
	void *cur_pos;
}AVS_CAFFE_BUF;

void *AVS_CAFFE_COM_alloc_buffer(	AVS_CAFFE_BUF *avs_buf,
									int size);

#endif
