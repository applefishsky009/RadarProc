#include "Avs_Radar_Interpolation_lib.h"

// opencv的warp_perspective C语言实现
void warp_perspective(	AVS_RADAR_PREPROCESS_IN *inbuf,
						AVS_RADAR_FILTER		*avs_filter) {
	int i = 0, j = 0, index = 0;
	int src_width	= inbuf->infrared_frame_info.width;
	int src_height	= inbuf->infrared_frame_info.height;
	int dst_width	= inbuf->frame_info.width;
	int dst_height	= inbuf->frame_info.height;
	double m11		= inbuf->infrared_calib_info.m11;
	double m12		= inbuf->infrared_calib_info.m12;
	double m13		= inbuf->infrared_calib_info.m13;
	double m21		= inbuf->infrared_calib_info.m21;
	double m22		= inbuf->infrared_calib_info.m22;
	double m23		= inbuf->infrared_calib_info.m23;
	double m31		= inbuf->infrared_calib_info.m31;
	double m32		= inbuf->infrared_calib_info.m32;
	double m33		= inbuf->infrared_calib_info.m33;
	float *src_data = inbuf->infrared_frame_info.data;
	float *dst_data = avs_filter->infrared_map.data;
	avs_filter->infrared_map.width		= dst_width;
	avs_filter->infrared_map.height		= dst_height;
	avs_filter->infrared_map.channels	= AVS_INFRARED_CHANNEL;
	for (int j = 0; j < src_height; j++) {	// 空图
		for (int i = 0; i < src_width; i++) {
			int index = j * src_width + i;	// 原图坐标
			// 计算目标映射图的坐标
			int d_x = (m11 * i + m12 * j + m13) / (m31 * i + m32 * j + m33) + 0.5;
			int d_y = (m21 * i + m22 * j + m23) / (m31 * i + m32 * j + m33) + 0.5;
			// 赋值
			int index_dst = d_y * dst_width + d_x;
			dst_data[index_dst] = src_data[index];
		}
	}
}

void infrared_bilinear_interpolation_im(	AVS_RADAR_PREPROCESS_IN *inbuf,
											AVS_RADAR_FILTER		*avs_filter) {
	int i = 0, j = 0, index = 0;
	float w_scale = inbuf->frame_info.width * 1.f / AVS_PROC_WIDTH;
	float h_scale = inbuf->frame_info.height * 1.f / AVS_PROC_HEIGHT;
	float *src_data = avs_filter->infrared_map.data;
	float *dst_data = avs_filter->infrared_resize_map.data;
	float i_scale = 0.f, j_scale = 0.f;
	for (j = 0; j < AVS_PROC_HEIGHT; ++j) {
		for (i = 0; i < AVS_PROC_WIDTH; ++i) {
			j_scale = h_scale * j;
			i_scale = w_scale * i;
			index = j * AVS_PROC_WIDTH + i;
			dst_data[index] = get_scale_value(src_data, i_scale, j_scale, inbuf->frame_info.width, inbuf->frame_info.height);
		}
	}
}

void bilinear_interpolation_im(	AVS_RADAR_PREPROCESS_IN *inbuf, 
								AVS_RADAR_FILTER		*avs_filter) {
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

void bilinear_output_info(	AVS_RADAR_FILTER					*avs_filter,
							AVS_RADAR_INTERPOLATIONPROCESS_OUT	*outbuf) {
	avs_filter->resize_map.width	= AVS_PROC_WIDTH;
	avs_filter->resize_map.height	= AVS_PROC_HEIGHT;
	avs_filter->resize_map.channels = AVS_RADAR_INPUT_CHANNEL;
	avs_filter->infrared_resize_map.width		= AVS_PROC_WIDTH;
	avs_filter->infrared_resize_map.height		= AVS_PROC_HEIGHT;
	avs_filter->infrared_resize_map.channels	= AVS_INFRARED_CHANNEL;
	// rgb数据输出
	outbuf->rgb_info.width			= AVS_PROC_WIDTH;
	outbuf->rgb_info.height			= AVS_PROC_HEIGHT;
	outbuf->rgb_info.channels		= AVS_RADAR_INPUT_CHANNEL;
	outbuf->rgb_info.data			= avs_filter->resize_map.data;
	// 红外数据输出
	outbuf->infrared_info.width		= AVS_PROC_WIDTH;
	outbuf->infrared_info.height	= AVS_PROC_HEIGHT;
	outbuf->infrared_info.channels	= AVS_INFRARED_CHANNEL;
	outbuf->infrared_info.data		= avs_filter->infrared_resize_map.data;
}
