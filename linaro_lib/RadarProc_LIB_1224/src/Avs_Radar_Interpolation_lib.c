#include "Avs_Radar_Interpolation_lib.h"

void bilinear_interpolation_im(	AVS_RADAR_PREPROCESS_IN *inbuf, 
								AVS_RADAR_FILTER *avs_filter) {
	int i = 0, j = 0, index = 0;
	int offset_src = inbuf->frame_info.width * inbuf->frame_info.height;
	int offset_resize = AVS_PROC_WIDTH * AVS_PROC_HEIGHT;
	float w_scale = inbuf->frame_info.width * 1.f / AVS_PROC_WIDTH;
	float h_scale = inbuf->frame_info.height * 1.f / AVS_PROC_HEIGHT;
	float *src_data_R = avs_filter->undistort_map.data;
	float *src_data_G = avs_filter->undistort_map.data + offset_src;
	float *src_data_B = avs_filter->undistort_map.data + 2 * offset_src;
	float *resize_data_R = avs_filter->resize_map.data;
	float *resize_data_G = avs_filter->resize_map.data + offset_resize;
	float *resize_data_B = avs_filter->resize_map.data + 2 * offset_resize;
	float i_scale = 0.f, j_scale = 0.f;
	for (j = 0; j < AVS_PROC_HEIGHT; ++j) {
		for (i = 0; i < AVS_PROC_WIDTH; ++i) {
			j_scale = h_scale * j;
			i_scale = w_scale * i;
			index = j * AVS_PROC_WIDTH + i;
			resize_data_R[index] = get_scale_value(src_data_R, i_scale, j_scale, inbuf->frame_info.width, inbuf->frame_info.height);
			resize_data_G[index] = get_scale_value(src_data_G, i_scale, j_scale, inbuf->frame_info.width, inbuf->frame_info.height);
			resize_data_B[index] = get_scale_value(src_data_B, i_scale, j_scale, inbuf->frame_info.width, inbuf->frame_info.height);
		}
	}
}

void bilinear_output_info(	AVS_RADAR_FILTER			*avs_filter,
							AVS_RADAR_PREPROCESS_OUT	*outbuf) {
	avs_filter->resize_map.width = AVS_PROC_WIDTH;
	avs_filter->resize_map.height = AVS_PROC_HEIGHT;
	avs_filter->resize_map.channels = AVS_RADAR_INPUT_CHANNEL;
	outbuf->width = AVS_PROC_WIDTH;
	outbuf->height = AVS_PROC_HEIGHT;
	outbuf->channels = AVS_RADAR_INPUT_CHANNEL;
	outbuf->data = avs_filter->resize_map.data;
}
