#ifndef _AVS_RADAR_LIB_H_
#define _AVS_RADAR_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "Avs_Base.h"
#include "Avs_Radar.h"
#include "Avs_Common.h"
#include "Avs_Radar_Common_lib.h"

#define RADAR_DEBUG_INFO_V2 0
#define RADAR_DEBUG_INFO_V3	0

#define AVS_PROC_SRC_WIDTH	1920
#define AVS_PROC_SRC_HEIGHT	1080
#define AVS_PROC_WIDTH		640
#define AVS_PROC_HEIGHT		384
#define AVS_PROC_ALIGN		4	// 输入图像的对齐位数要求
#define AVS_EPS				0.00001
#define AVS_DEBUG_CACHE		13
#define AVS_DEBUG_POINT_NUM	125

typedef struct _AVS_RADAR_POINT_PRE_INFO_ {
	int			valid;
	float		dis;
	float		arm;
	float		vel;
	float		vrad_comp;
	AVS_POINT_F radar_top;
	AVS_POINT_F radar_bottom;
	AVS_POINT_F radar_top_world;
	AVS_POINT_F radar_bottom_world;
	AVS_POINT_F radar_top_image;
	AVS_POINT_F radar_bottom_image;
	AVS_POINT_F radar_top_image_debug;
	AVS_POINT_F radar_bottom_image_debug;
}AVS_RADAR_POINT_PRE_INFO;

typedef struct _AVS_RADAR_PREINFO_ {
	int							radar_point_num;	// 缓存过去不同时刻的雷达信息
	AVS_RADAR_POINT_PRE_INFO	*radar_cal_info;	// 确定时刻的雷达信息，0是最近的信息
}AVS_RADAR_PREINFO;

typedef struct _AVS_RADAR_FILTER_ {
	int							radar_info_cache_num;
	AVS_RADAR_PREINFO			radar_pre_info[AVS_MAX_RADAR_CACHE];	// 由雷达信息预处理得到我们训练需要的信息
	
	int							valid_box_num;
	AVS_RADAR_POSTINFO			*radar_post_info[AVS_POST_OUT_LENGTH];
	AVS_RADAR_PREPROCESS_OUT	undistort_map;
	AVS_RADAR_PREPROCESS_OUT	infrared_map;
	AVS_RADAR_PREPROCESS_OUT	resize_map;
	AVS_RADAR_PREPROCESS_OUT	infrared_resize_map;
}AVS_RADAR_FILTER;

void cal_radar_distri(AVS_RADAR_PREPROCESS_IN *inbuf, AVS_RADAR_FILTER *avs_filter);

void cal_radarInImage(AVS_RADAR_PREPROCESS_IN *inbuf, AVS_RADAR_FILTER *avs_filter);

#if RADAR_DEBUG_INFO_V2
void cal_radarinimage_v2(AVS_RADAR_PREPROCESS_IN *inbuf, AVS_RADAR_FILTER *avs_filter);
#endif

#if RADAR_DEBUG_INFO_V3
void cal_radar_distri_v3(	AVS_RADAR_PREPROCESS_IN *inbuf,
							AVS_RADAR_FILTER		*avs_filter);

void cal_radarinimage_v3(	AVS_RADAR_PREPROCESS_IN *inbuf, 
							AVS_RADAR_FILTER		*avs_filter);
#endif

void cal_output_info(	AVS_RADAR_PREPROCESS_IN		*inbuf,
						AVS_RADAR_FILTER			*avs_filter,
						AVS_RADAR_PREPROCESS_OUT	*outbuf);

void normalize_input(	AVS_RADAR_FILTER *inbuf);

#ifdef __cplusplus
}
#endif

#endif