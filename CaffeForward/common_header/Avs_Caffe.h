#ifndef _AVS_CAFFE_H_
#define _AVS_CAFFE_H_

#include "Avs_Radar.h"

typedef struct _AVS_CAFFE_INFO_ {
	char *proto;
	char *model;
	AVS_RADAR_PREPROCESS_OUT caffe_input;
}AVS_CAFFE_INFO;

unsigned int AVS_Caffe_GetMemSize(	AVS_CAFFE_INFO			*input,
									AVS_MEM_TAB				mem_tab[AVS_RADAR_MEM_TAB]);

unsigned int AVS_Caffe_CreatMemSize(	AVS_CAFFE_INFO			*input,
										AVS_MEM_TAB				mem_tab[AVS_RADAR_MEM_TAB],
										void					**handle);

unsigned int AVS_Caffe_Init(void						*handle, 
							AVS_CAFFE_INFO				*inbuf,
							int							in_buf_size);

unsigned int AVS_Caffe_Forward(	void						*handle,
								AVS_CAFFE_INFO				*inbuf,
								int							in_buf_size,
								AVS_RADAR_POSTPROCESS_IN	*outbuf,
								int							out_buf_size);

#endif