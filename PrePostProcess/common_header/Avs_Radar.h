#ifndef _AVS_RADAR_H_
#define _AVS_RADAR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Avs_Base.h"
#include "Avs_Common.h"
#include "Avs_ErrorCode.h"

#define AVS_MAX_RADAR_NUM			125
#define AVS_RADAR_MEM_TAB			1
#define AVS_MAX_RADAR_CACHE			20
#define AVS_POST_OUT_LENGTH			46035
#define AVS_POST_OUT_MXA_OBJ		128
#define AVS_INFRARED_CHANNEL		1
#define AVS_RADAR_INPUT_CHANNEL		3
#define AVS_RADAR_OUTPUT_CHANNEL	5
#define AVS_MAX_BUF_LEN				1024

typedef struct _AVS_FRAME_INFO_ {
	int frame_num;
	int width;
	int height;
	int channel;
	float *data;
}AVS_FRAME_INFO;

// 雷达点结构信息
typedef struct _AVS_RADAR_POINT_INFO_{
	unsigned int id;
	unsigned int dyn_prop;	// Dynamic property of cluster to indicate if is moving or not.
	float rcs;
	int is_quality_valid;
	int ambig_state;	// State of Doppler (radial velocity) ambiguity solution.
	int invalid_state;	// state of Cluster validity state.
	int phd0;	// False alarm probability of cluster (i.e. probability of being an artefact caused by multipath or similar).
	AVS_POINT_F radar_point; // (x, y, z)
	AVS_POINT_F radar_v; // 速度 (vx, vy)
	AVS_POINT_F radar_v_comp; // 补偿速度 (vx_comp, vy_comp)
	AVS_POINT_F radar_rms;
	AVS_POINT_F radar_v_rms;
}AVS_RADAR_POINT_INFO;

typedef struct _AVS_RADAR_INFO_ {
	int radar_point_num;	// 当前扫描时刻的雷达点数量
	AVS_RADAR_POINT_INFO* radar_point_info;	// 雷达点信息
}AVS_RADAR_INFO;

// 相机结构化信息
typedef struct _AVS_CAMERA_INFO_ {
	float fu;	// 相机内参
	float fv;
	float cu;
	float cv;
	float pitch;	// 外参
	float yaw;
	float roll;
	float height;	// 相机高度 mm
	float k1;	// 畸变系数
	float k2;
	float k3;
	float p1;
	float p2;
}AVS_CAMERA_INFO;

// 相机结构化信息
typedef struct _AVS_RADAR_CALIB_ {
	float radar_radian;	// 雷达在相机坐标系中的横摆角
	float radarInCamera_X; // 雷达在相机坐标系中的X平移 mm
	float radarInCamera_Y; // 雷达在相机坐标系中的Y平移 mm
	float radar_height; // 雷达架设高度 mm
	float radar_top;    // 假设物体的真实高度，用于雷达像像素平面映射 mm
}AVS_RADAR_CALIB;

// 调试信息输入
typedef struct _AVS_RADAR_DEBUG_INFO_ {
	FILE *fp[AVS_MAX_RADAR_CACHE];
	FILE *fp_point;
	FILE *fp_point_xyz;
	FILE *out_map;
}AVS_RADAR_DEBUG_INFO;

// 畸变校正参数
typedef struct _AVS_IMAGE_CONFIG_ {
	int undistort_avalible;
	int infrared_align;
}AVS_IMAGE_CONFIG;

typedef struct _AVS_INFRARED_INFO_ {
	double m11;
	double m12;
	double m13;
	double m21;
	double m22;
	double m23;
	double m31;
	double m32;
	double m33;
}AVS_INFRARED_INFO;

typedef struct _AVS_RADAR_PREPROCESS_IN_ {
	int						radar_info_cache_num;	// 缓存过去不同时刻的雷达信息
	AVS_RADAR_INFO			radar_info[AVS_MAX_RADAR_CACHE];	// 确定时刻的雷达信息，0是最近的信息
	AVS_FRAME_INFO			frame_info;				// 当前的适配帧-rgb
	AVS_FRAME_INFO			infrared_frame_info;	// 当前的适配帧-红外
	AVS_CAMERA_INFO			cam_info;				// 相机参数
	AVS_INFRARED_INFO		infrared_calib_info;	// 红外仿射变换矩阵参数
	AVS_RADAR_CALIB			radar_calib;			// 相机参数
	AVS_IMAGE_CONFIG		image_config;			// 配置信息
	AVS_RADAR_DEBUG_INFO	debug_info;				// 输入调试信息
}AVS_RADAR_PREPROCESS_IN;

typedef struct _AVS_RADAR_PREPROCESS_OUT_ {
	int width;
	int height;
	int channels;
	float *data;
}AVS_RADAR_PREPROCESS_OUT;

typedef struct _AVS_RADAR_INTERPOLATIONPROCESS_OUT_ {
	AVS_RADAR_PREPROCESS_OUT rgb_info;
	AVS_RADAR_PREPROCESS_OUT infrared_info;
}AVS_RADAR_INTERPOLATIONPROCESS_OUT;

typedef struct _AVS_RADAR_POSTPROCESS_CLASSIFICATION_ {
	float conf[AVS_RADAR_OBJ_END];
}AVS_RADAR_POSTPROCESS_CLASSIFICATION;

typedef struct _AVS_RADAR_POSTPROCESS_REGRESSION_ {
	float x1;
	float y1;
	float x2;
	float y2;
}AVS_RADAR_POSTPROCESS_REGRESSION;

// 调试信息输入
typedef struct _AVS_RADAR_POST_DEBUG_INFO_ {
	FILE *classification;
	FILE *regression;
}AVS_RADAR_POST_DEBUG_INFO;

typedef struct _AVS_RADAR_POSTPROCESS_IN_ {
	AVS_RADAR_POST_DEBUG_INFO			 post_debug_info;	// 输入调试信息
	AVS_RADAR_POSTPROCESS_CLASSIFICATION *classification[AVS_POST_OUT_LENGTH];
	AVS_RADAR_POSTPROCESS_REGRESSION	 *regression[AVS_POST_OUT_LENGTH];
}AVS_RADAR_POSTPROCESS_IN;

typedef struct _AVS_RADAR_POSTINFO_ {
	int		valid;
	int		x1;
	int		y1;
	int		x2;
	int		y2;
	int		cls;
	float	confidence;
	float	velocity;
	float	distance;
	float	temp;
	float	temp_pixel;
}AVS_RADAR_POSTINFO;

typedef struct _AVS_RADAR_POSTPROCESS_OUT_ {
	int					obj_num;
	AVS_RADAR_POSTINFO	*obj_info[AVS_POST_OUT_MXA_OBJ];
}AVS_RADAR_POSTPROCESS_OUT;

typedef struct _AVS_RADAR_PROCESS_IN_ {
	AVS_RADAR_PREPROCESS_IN pre_input;
	AVS_RADAR_POSTPROCESS_IN post_input;
}AVS_RADAR_PROCESS_IN;

unsigned int AVS_GetMemSize(AVS_RADAR_PROCESS_IN	*radar_param,
							AVS_MEM_TAB				memtab[AVS_RADAR_MEM_TAB]);

unsigned int AVS_CreatMemSize(	AVS_RADAR_PROCESS_IN	*radar_param,
								AVS_MEM_TAB				memtab[AVS_RADAR_MEM_TAB],
								void					**handle);

unsigned int AVS_UndistortProcess(	void						*handle,
									AVS_RADAR_PREPROCESS_IN		*inbuf,
									int							in_buf_size,
									AVS_RADAR_PREPROCESS_OUT	*outbuf,
									int							out_buf_size);

unsigned int AVS_InterpolationProcess(	void								*handle,
										AVS_RADAR_PREPROCESS_IN				*inbuf,
										int									in_buf_size,
										AVS_RADAR_INTERPOLATIONPROCESS_OUT	*outbuf,
										int									out_buf_size);

unsigned int AVS_PreProcess(	void						*handle, 
								AVS_RADAR_PREPROCESS_IN		*inbuf,
								int							in_buf_size,
								AVS_RADAR_PREPROCESS_OUT	*outbuf,
								int							out_buf_size);

unsigned int AVS_PostProcess(	void						*handle,
								AVS_RADAR_POSTPROCESS_IN	*inbuf, 
								int							in_buf_size,
								AVS_RADAR_POSTPROCESS_OUT	*outbuf,
								int							out_buf_size);

unsigned int AVS_SBProcess(		void						*handle,
								AVS_RADAR_POSTPROCESS_OUT	*inbuf,
								int							in_buf_size,
								AVS_RADAR_POSTPROCESS_OUT	*outbuf,
								int							out_buf_size);

#ifdef __cplusplus
}
#endif

#endif