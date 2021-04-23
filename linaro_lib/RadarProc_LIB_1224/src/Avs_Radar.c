#include "Avs_Common.h"
#include "Avs_ErrorCode.h"
#include "Avs_Radar.h"
#include "Avs_Radar_lib.h"
#include "Avs_Radar_Post_lib.h"
#include "Avs_Radar_Common_lib.h"
#include "Avs_Radar_Undistort_lib.h"
#include "Avs_Radar_Interpolation_lib.h"
#include <math.h>

unsigned int AVS_GetMemSize(AVS_RADAR_PROCESS_IN	*input,
							AVS_MEM_TAB				mem_tab[AVS_RADAR_MEM_TAB]) {
	unsigned int mem_size = 0;
	AVS_RADAR_PREPROCESS_IN *pre_input = &input->pre_input;
	AVS_RADAR_POSTPROCESS_IN *post_input = &input->post_input;

	AVS_CHECK_ERROR(pre_input->frame_info.width > AVS_PROC_SRC_WIDTH, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(pre_input->frame_info.height > AVS_PROC_SRC_HEIGHT, AVS_LIB_RESOLUTION_UNSUPPORT);

	// 算法库句柄内存
	mem_size += sizeof(AVS_RADAR_FILTER) + AVS_MEM_ALIGN_128BYTE;

	// 输入图像内存 位于pre_input
	mem_size += pre_input->frame_info.width * pre_input->frame_info.height * sizeof(float) * AVS_RADAR_INPUT_CHANNEL + AVS_MEM_ALIGN_128BYTE;
	// 畸变矫正后的图像内存 位于filter
	mem_size += pre_input->frame_info.width * pre_input->frame_info.height * sizeof(float) * AVS_RADAR_INPUT_CHANNEL + AVS_MEM_ALIGN_128BYTE;
	// 降采样后的图像内存 位于filter
	mem_size += AVS_PROC_WIDTH * AVS_PROC_HEIGHT * sizeof(float) * AVS_RADAR_OUTPUT_CHANNEL + AVS_MEM_ALIGN_128BYTE;

	// 输入雷达信息内存 位于pre_input
	mem_size += (sizeof(AVS_RADAR_POINT_INFO) * AVS_MAX_RADAR_NUM + AVS_MEM_ALIGN_128BYTE) * AVS_MAX_RADAR_CACHE;
	// 雷达点预处理信息内存 位于filter
	mem_size += (sizeof(AVS_RADAR_POINT_PRE_INFO) * AVS_MAX_RADAR_NUM + AVS_MEM_ALIGN_128BYTE) * AVS_MAX_RADAR_CACHE;
	
	// 模型输出分类信息 classification
	mem_size += (sizeof(AVS_RADAR_POSTPROCESS_CLASSIFICATION) + AVS_MEM_ALIGN_128BYTE) * AVS_POST_OUT_LENGTH;
	// 模型输出回归信息 regression
	mem_size += (sizeof(AVS_RADAR_POSTPROCESS_REGRESSION) + AVS_MEM_ALIGN_128BYTE) * AVS_POST_OUT_LENGTH;

	// 数据后处理的boxes生成内存 - AVS_RADAR_FILTER - AVS_RADAR_POSTINFO
	mem_size += (sizeof(AVS_RADAR_POSTINFO) + AVS_MEM_ALIGN_128BYTE) * AVS_POST_OUT_LENGTH;
	
	mem_tab[0].size = mem_size;
	mem_tab[0].base = NULL;
	mem_tab[0].alignment = AVS_MEM_ALIGN_128BYTE;
	return 0;
}

unsigned int AVS_CreatMemSize(	AVS_RADAR_PROCESS_IN	*input,
								AVS_MEM_TAB				mem_tab[AVS_RADAR_MEM_TAB],
								void					**handle) {
	int i = 0;
	AVS_RADAR_FILTER *filter = NULL;
	AVS_RADAR_PREPROCESS_IN *pre_input = &input->pre_input;
	AVS_RADAR_POSTPROCESS_IN *post_input = &input->post_input;
	AVS_BUF mem_buf;
	mem_buf.start	= mem_tab[0].base;
	mem_buf.cur_pos = mem_tab[0].base;
	mem_buf.end		= (void *)((QWORD)mem_buf.cur_pos + (QWORD)mem_tab[0].size);
	
	AVS_CHECK_ERROR(pre_input->frame_info.width > AVS_PROC_SRC_WIDTH, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(pre_input->frame_info.width % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(pre_input->frame_info.height > AVS_PROC_SRC_HEIGHT, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(pre_input->frame_info.height % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(pre_input->frame_info.channel != 3, AVS_LIB_RESOLUTION_UNSUPPORT);

	// 算法库句柄内存
	filter = (AVS_RADAR_FILTER *)AVS_COM_alloc_buffer(&mem_buf, sizeof(AVS_RADAR_FILTER));

	// 输入图像内存 位于 pre_input
	pre_input->frame_info.data = (unsigned char *)AVS_COM_alloc_buffer(&mem_buf, pre_input->frame_info.width * pre_input->frame_info.height * sizeof(float) * AVS_RADAR_INPUT_CHANNEL);
	// 畸变矫正后的图像内存 位于filter
	filter->undistort_map.data = (unsigned char *)AVS_COM_alloc_buffer(&mem_buf, pre_input->frame_info.width * pre_input->frame_info.height * sizeof(float) * AVS_RADAR_INPUT_CHANNEL);
	// 双线性插值降采样后的图像内存 位于filter
	filter->resize_map.data = (unsigned char *)AVS_COM_alloc_buffer(&mem_buf, AVS_PROC_WIDTH * AVS_PROC_HEIGHT * sizeof(float) * AVS_RADAR_OUTPUT_CHANNEL);
	// 输入雷达信息内存 位于 pre_input
	for (i = 0; i < AVS_MAX_RADAR_CACHE; ++i) {
		pre_input->radar_info[i].radar_point_info = (AVS_RADAR_POINT_INFO *)AVS_COM_alloc_buffer(&mem_buf, sizeof(AVS_RADAR_POINT_INFO) * AVS_MAX_RADAR_NUM);
	}
	// 雷达点预处理信息内存 位于 filter
	for (i = 0; i < AVS_MAX_RADAR_CACHE; ++i) {
		filter->radar_pre_info[i].radar_cal_info = (AVS_RADAR_POINT_PRE_INFO *)AVS_COM_alloc_buffer(&mem_buf, sizeof(AVS_RADAR_POINT_PRE_INFO) * AVS_MAX_RADAR_NUM);
	}

	// 模型输出分类信息内存 classification
	for (i = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		post_input->classification[i] = (AVS_RADAR_POSTPROCESS_CLASSIFICATION *)AVS_COM_alloc_buffer(&mem_buf, sizeof(AVS_RADAR_POSTPROCESS_CLASSIFICATION));
	}
	// 模型输出回归信息内存 regression
	for (i = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		post_input->regression[i] = (AVS_RADAR_POSTPROCESS_REGRESSION *)AVS_COM_alloc_buffer(&mem_buf, sizeof(AVS_RADAR_POSTPROCESS_REGRESSION));
	}

	// 数据后处理的boxes生成内存 - AVS_RADAR_FILTER - AVS_RADAR_POSTINFO
	for (i = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		filter->radar_post_info[i] = (AVS_RADAR_POSTINFO *)AVS_COM_alloc_buffer(&mem_buf, sizeof(AVS_RADAR_POSTINFO));
	}

	*handle = (void *)filter;
	return 0;
}

// 畸变矫正
unsigned int AVS_UndistortProcess(	void						*handle,
									AVS_RADAR_PREPROCESS_IN		*inbuf,
									int							in_buf_size,
									AVS_RADAR_PREPROCESS_OUT	*outbuf,
									int							out_buf_size) {
	
	int i = 0, j = 0;
	AVS_RADAR_FILTER *avs_filter = (AVS_RADAR_FILTER *)handle;

	// 输入效验
	AVS_CHECK_ERROR(inbuf->frame_info.width > AVS_PROC_SRC_WIDTH, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.width % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.height > AVS_PROC_SRC_HEIGHT, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.height % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.channel != 3, AVS_LIB_RESOLUTION_UNSUPPORT);

	// 进行畸变校正
	undistort_im_bi(inbuf, avs_filter);
	// undistort_im_nearest(inbuf, avs_filter);
	
	// 产生输出信息
	undistort_output_info(inbuf, avs_filter, outbuf);

	return 0;
}

// 插值算法降采样
unsigned int AVS_InterpolationProcess(	void						*handle,
										AVS_RADAR_PREPROCESS_IN		*inbuf,
										int							in_buf_size,
										AVS_RADAR_PREPROCESS_OUT	*outbuf,
										int							out_buf_size) {

	int i = 0, j = 0;
	AVS_RADAR_FILTER *avs_filter = (AVS_RADAR_FILTER *)handle;

	// 输入效验
	AVS_CHECK_ERROR(inbuf->frame_info.width > AVS_PROC_SRC_WIDTH, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.width % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.height > AVS_PROC_SRC_HEIGHT, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.height % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.channel != 3, AVS_LIB_RESOLUTION_UNSUPPORT);

	// 进行双线性插值
	bilinear_interpolation_im(inbuf, avs_filter);

	// 产生输出信息
	bilinear_output_info(avs_filter, outbuf);

	return 0;
}

// 雷达数据预处理
unsigned int AVS_PreProcess(void						*handle,
							AVS_RADAR_PREPROCESS_IN		*inbuf,
							int							in_buf_size,
							AVS_RADAR_PREPROCESS_OUT	*outbuf,
							int							out_buf_size) {
	
	int i = 0, j = 0;
	AVS_RADAR_FILTER *avs_filter = (AVS_RADAR_FILTER *)handle;

	// 输入效验
	AVS_CHECK_ERROR(inbuf->frame_info.width > AVS_PROC_SRC_WIDTH, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.width % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.height > AVS_PROC_SRC_HEIGHT, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.height % AVS_PROC_ALIGN != 0, AVS_LIB_RESOLUTION_UNSUPPORT);
	AVS_CHECK_ERROR(inbuf->frame_info.channel != 3, AVS_LIB_RESOLUTION_UNSUPPORT);

	// 对resize图像进行normalization
	normalize_input(avs_filter);

	
#if RADAR_DEBUG_INFO_V2
	// 调试信息
	cal_radarinimage_v2(inbuf, avs_filter);
#elif RADAR_DEBUG_INFO_V3
	// 计算距离，方位角，速度直径补偿
	cal_radar_distri_v3(inbuf, avs_filter);
	// 调试信息
	cal_radarinimage_v3(inbuf, avs_filter);
#else
	// 计算距离，方位角，速度直径补偿
	cal_radar_distri(inbuf, avs_filter);
	// 根据相机内参将点投影到像素平面
	cal_radarInImage(inbuf, avs_filter);
#endif

	// 产生输出信息
	cal_output_info(inbuf, avs_filter, outbuf);

	return 0;
}

// 雷达数据后处理
unsigned int AVS_PostProcess(	void						*handle,
								AVS_RADAR_POSTPROCESS_IN	*inbuf,
								int							in_buf_size,
								AVS_RADAR_POSTPROCESS_OUT	*outbuf,
								int							out_buf_size) {
	int i = 0, j = 0;
	AVS_RADAR_FILTER *avs_filter = (AVS_RADAR_FILTER *)handle;

#if RADAR_DEBUG_INFO_V3
	read_cls_reg(inbuf, avs_filter);
#endif

	// anchor回归计算
	anchor_regression(inbuf, avs_filter);

	// 根据置信度排序
	anchor_sort(inbuf, avs_filter);

	// anchor nms计算，保留300个
	anchor_nms_all_cls(inbuf, avs_filter);

	// 产生输出信息
	cal_post_output_info(inbuf, avs_filter, outbuf);

	// 为输出信息匹配速度和距离信息
	match_attribute(avs_filter, outbuf);

	return 0;
}
