#include "Avs_Radar_Post_lib.h"

void read_cls_reg(		AVS_RADAR_POSTPROCESS_IN	*inbuf,
						AVS_RADAR_FILTER			*avs_filter) {
	int i = 0, j = 0;
	FILE *fp_cls = inbuf->post_debug_info.classification;
	FILE *fp_reg = inbuf->post_debug_info.regression;
	char *token;
	char buf[AVS_MAX_BUF_LEN * 10];
	// 读分类信息
	for (i = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		memset(buf, 0, AVS_MAX_BUF_LEN * 10);
		fgets(buf, AVS_MAX_BUF_LEN * 10, fp_cls);
		token = strtok(buf, " ");	// 0
		inbuf->classification[i]->conf[0] = atof(token);
		token = strtok(NULL, " ");	// 1
		inbuf->classification[i]->conf[1] = atof(token);
		token = strtok(NULL, " ");	// 2
		inbuf->classification[i]->conf[2] = atof(token);
		token = strtok(NULL, " ");	// 3
		inbuf->classification[i]->conf[3] = atof(token);
		token = strtok(NULL, " ");	// 4
		inbuf->classification[i]->conf[4] = atof(token);
		token = strtok(NULL, " ");	// 5
		inbuf->classification[i]->conf[5] = atof(token);
		token = strtok(NULL, " ");	// 6
		inbuf->classification[i]->conf[6] = atof(token);
		token = strtok(NULL, " ");	// 7
		inbuf->classification[i]->conf[7] = atof(token);
		int kkk = 0;
	}
	// 读回归信息
	for (i = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		memset(buf, 0, AVS_MAX_BUF_LEN * 10);
		fgets(buf, AVS_MAX_BUF_LEN * 10, fp_reg);
		token = strtok(buf, " ");	// 0
		inbuf->regression[i]->x1 = atof(token);
		token = strtok(NULL, " ");	// 1
		inbuf->regression[i]->y1 = atof(token);
		token = strtok(NULL, " ");	// 2
		inbuf->regression[i]->x2 = atof(token);
		token = strtok(NULL, " ");	// 3
		inbuf->regression[i]->y2 = atof(token);
		int kkk = 0;
	}
}

void anchor_regression(	AVS_RADAR_POSTPROCESS_IN	*inbuf, 
						AVS_RADAR_FILTER			*avs_filter) {
	int i = 0, j = 0;
	float w = 0.f, h = 0.f;
	avs_filter->valid_box_num = 0;
	for (i = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		// 计算坐标
		w = out_anchors[i][2] - out_anchors[i][0];
		h = out_anchors[i][3] - out_anchors[i][1];
		int x1 = (int)(out_anchors[i][0] + inbuf->regression[i]->x1 * AVS_POST_STD * w + 0.5);
		int y1 = (int)(out_anchors[i][1] + inbuf->regression[i]->y1 * AVS_POST_STD * h + 0.5);
		int x2 = (int)(out_anchors[i][2] + inbuf->regression[i]->x2 * AVS_POST_STD * w + 0.5);
		int y2 = (int)(out_anchors[i][3] + inbuf->regression[i]->y2 * AVS_POST_STD * h + 0.5);
		// 计算类别和置信度
		int cls_index = -1;
		float cls_conf  = 0;
		for (j = 0; j < AVS_RADAR_OBJ_END; ++j) {
			if (inbuf->classification[i]->conf[j] > cls_conf) {
				cls_index = j;
				cls_conf = inbuf->classification[i]->conf[j];
			}
		}
		// 根据置信度过滤, 有效位标识更新
		AVS_CHECK_CONTINUE(cls_conf < AVS_NMS_CONF);
		// AVS_CHECK_CONTINUE(cls_index == AVS_RADAR_OBJ_BACKGROUND);
		avs_filter->radar_post_info[avs_filter->valid_box_num]->x1			= AVS_MAX(0, x1);
		avs_filter->radar_post_info[avs_filter->valid_box_num]->y1			= AVS_MAX(0, y1);
		avs_filter->radar_post_info[avs_filter->valid_box_num]->x2			= AVS_MIN(x2, AVS_PROC_WIDTH);
		avs_filter->radar_post_info[avs_filter->valid_box_num]->y2			= AVS_MIN(y2, AVS_PROC_HEIGHT);
		avs_filter->radar_post_info[avs_filter->valid_box_num]->valid		= 1;
		avs_filter->radar_post_info[avs_filter->valid_box_num]->cls			= cls_index;
		avs_filter->radar_post_info[avs_filter->valid_box_num]->confidence	= cls_conf;
		++avs_filter->valid_box_num;
	}
}

// 位置交换
void anchor_swap(AVS_RADAR_POSTINFO *dst, AVS_RADAR_POSTINFO *src) {
	AVS_RADAR_POSTINFO tmp;
	memcpy(&tmp, src, sizeof(AVS_RADAR_POSTINFO));
	memcpy(src, dst, sizeof(AVS_RADAR_POSTINFO));
	memcpy(dst, &tmp, sizeof(AVS_RADAR_POSTINFO));
}

// 矩形框iou计算
float anchor_iou(AVS_RADAR_POSTINFO *dst, AVS_RADAR_POSTINFO *src) {
	int x1 = AVS_MAX(dst->x1, src->x1);
	int y1 = AVS_MAX(dst->y1, src->y1);
	int x2 = AVS_MIN(dst->x2, src->x2);
	int y2 = AVS_MIN(dst->y2, src->y2);
	int w = x2 - x1, h = y2 - y1;
	if (w < 0 || h < 0) {
		return 0;
	}
	int cross = w * h;
	int area1 = (src->x2 - src->x1) * (src->y2 - src->y1);
	int area2 = (dst->x2 - dst->x1) * (dst->y2 - dst->y1);
	float iou = cross / (area1 + area2 - cross + FLT_MIN);
	return iou;
}

// 冒泡排序
void anchor_sort(AVS_RADAR_POSTPROCESS_IN *inbuf, AVS_RADAR_FILTER *avs_filter) {
	int i = 0, j = 0;
	AVS_RADAR_POSTINFO *cur, *dst;
	for (i = 0; i < avs_filter->valid_box_num; ++i) {
		cur = avs_filter->radar_post_info[i];
		for (j = i + 1; j < avs_filter->valid_box_num; ++j) {
			dst = avs_filter->radar_post_info[j];
			if (dst->confidence > cur->confidence) {
				anchor_swap(dst, cur);
			}
		}
	}
}

// 单个类别的nms, 注意这个函数只把valid标识位改变了，对valid_box_num没有进行处理，该信息在输出的时候处理
void anchor_nms(AVS_RADAR_POSTPROCESS_IN *inbuf, AVS_RADAR_FILTER *avs_filter, int cls) {
	int i = 0, j = 0;
	float iou = 0.f;
	for (i = 0; i < avs_filter->valid_box_num; ++i) {
		//AVS_CHECK_CONTINUE(avs_filter->radar_post_info[i]->cls != cls);	// 同类别间进行nms
		AVS_CHECK_CONTINUE(avs_filter->radar_post_info[i]->valid != 1);	// 无效anchor不对其他anchor进行nms
		for (j = i + 1; j < avs_filter->valid_box_num; ++j) {
			//AVS_CHECK_CONTINUE(avs_filter->radar_post_info[j]->cls != cls);	// 同类别间进行nms
			AVS_CHECK_CONTINUE(avs_filter->radar_post_info[j]->valid != 1);	// 无效anchor不对其他anchor进行nms
			// 做NMS
			iou = anchor_iou(avs_filter->radar_post_info[i], avs_filter->radar_post_info[j]);
			if (iou > AVS_NMS_IOU_TH) {
				avs_filter->radar_post_info[j]->valid = 0;
			}
		}
	}
}

// 分类别nms
void anchor_nms_all_cls(AVS_RADAR_POSTPROCESS_IN *inbuf, 
						AVS_RADAR_FILTER *avs_filter) {
	//int i = 0;
	//for (i = 0; i < AVS_RADAR_OBJ_END; ++i) {
	//	anchor_nms(inbuf, avs_filter, i);
	//}
	anchor_nms(inbuf, avs_filter, 0);
}

// 输出信息处理 快慢指针
void cal_post_output_info(	AVS_RADAR_POSTPROCESS_IN	*inbuf, 
							AVS_RADAR_FILTER			*avs_filter,
							AVS_RADAR_POSTPROCESS_OUT	*outbuf) {
	int i = 0, j = 0;
	AVS_RADAR_POSTINFO *cur, *dst;
	// 删除无效目标和背景目标
	cur = avs_filter->radar_post_info[i];
	for (j = 0; j < avs_filter->valid_box_num; ++j) {
		dst = avs_filter->radar_post_info[j];
		if (dst->valid && dst->cls != AVS_RADAR_OBJ_BACKGROUND) {
			memcpy(cur, dst, sizeof(AVS_RADAR_POSTINFO));
			cur = avs_filter->radar_post_info[++i];
		}
	}
	avs_filter->valid_box_num = i;
	// 截断，最多输出64个目标
	outbuf->obj_num = AVS_MIN(avs_filter->valid_box_num, AVS_POST_OUT_MXA_OBJ);
	for (i = 0; i < outbuf->obj_num; ++i) {
		outbuf->obj_info[i] = avs_filter->radar_post_info[i];
	}
}

void match_attribute(	AVS_RADAR_FILTER			*avs_filter,
						AVS_RADAR_POSTPROCESS_OUT	*outbuf) {
	int i = 0, j = 0, k = 0, radar_point_num = 0;
	float c_x = 0.f, c_y = 0.f, c_rx = 0.f, c_ry = 0.f;
	float vel = 0.f, dis = 0.f, pixel_dis = FLT_MAX, pixel_dis_tmp = 0.f;
	for (i = 0; i < outbuf->obj_num; ++i) {
		// 计算目标中心点
		c_x = (outbuf->obj_info[i]->x2 - outbuf->obj_info[i]->x1) >> 1;
		c_y = (outbuf->obj_info[i]->y2 - outbuf->obj_info[i]->y1) >> 1;
		// 寻找距离目标中心点最近的雷达中心点 只寻找最近的一帧雷达
		pixel_dis = FLT_MAX, pixel_dis_tmp = 0.f;	// 初始化
		for (j = 0; j < avs_filter->radar_info_cache_num; ++j) {
			radar_point_num = avs_filter->radar_pre_info[j].radar_point_num;
			for (k = 0; k < radar_point_num; ++k) {
				// 被滤波的点无效
				// AVS_CHECK_CONTINUE(!avs_filter->radar_pre_info[j].radar_cal_info[k].valid);
				// 计算雷达中心点
				c_rx = ((int)avs_filter->radar_pre_info[j].radar_cal_info[k].radar_top_image.x + 
					(int)avs_filter->radar_pre_info[j].radar_cal_info[k].radar_bottom_image.x) >> 1;
				c_ry = ((int)avs_filter->radar_pre_info[j].radar_cal_info[k].radar_top_image.y + 
					(int)avs_filter->radar_pre_info[j].radar_cal_info[k].radar_bottom_image.y) >> 1;
				// 雷达点在画面外的无效
				AVS_CHECK_CONTINUE((c_rx < AVS_EPS && c_ry < AVS_EPS));
				pixel_dis_tmp = sqrt((c_rx - c_x) * (c_rx - c_x) + (c_ry - c_y) * (c_ry - c_y));
				//printf("%f %f %f \n", 
				//	c_rx,
				//	c_ry,
				//	avs_filter->radar_pre_info[j].radar_cal_info[k].dis);
				if (pixel_dis_tmp < pixel_dis) {
					pixel_dis = pixel_dis_tmp;
					vel = avs_filter->radar_pre_info[j].radar_cal_info[k].vel;
					dis = avs_filter->radar_pre_info[j].radar_cal_info[k].dis;
				}
			}
			int kkk = 0;
		}
		outbuf->obj_info[i]->velocity = vel;
		outbuf->obj_info[i]->distance = dis;
	}
}

void match_temp(AVS_RADAR_FILTER			*avs_filter,
				AVS_RADAR_POSTPROCESS_OUT	*outbuf) {
	int i = 0, j = 0, k = 0, index = 0;
	float temp = 0.f, temp_C = 0.f;
	AVS_RADAR_POSTINFO	*obj_info = NULL;
	int width	= avs_filter->infrared_resize_map.width;
	int height	= avs_filter->infrared_resize_map.height;
	float *data = avs_filter->infrared_resize_map.data;
	for (k = 0; k < outbuf->obj_num; ++k) {		// 针对所有目标查找最高温度
		obj_info = outbuf->obj_info[k];
		temp = 0.f;
		for (j = obj_info->y1; j < obj_info->y2; ++j) {
			for (i = obj_info->x1; i < obj_info->x2; ++i) {
				index = j * width + i;
				temp = AVS_MAX(temp, data[index]);
			}
		}
		temp_C = 0.08 * temp + 19.6;
		obj_info->temp_pixel = (int)(temp + 0.5);
		obj_info->temp = temp_C;
	}
}
