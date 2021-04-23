#include "Avs_Radar_Common_lib.h"

void *AVS_COM_alloc_buffer(AVS_BUF *avs_buf, int size) {
	void *buf;
	int free_size;
	// �����п����ڴ����ʼλ��
	buf = (void *)(((QWORD)avs_buf->cur_pos + (AVS_MEM_ALIGN_128BYTE - 1)) & (~(AVS_MEM_ALIGN_128BYTE - 1)));
	// ���㻺����ʣ��ռ��С
	free_size = (QWORD)avs_buf->end - (QWORD)buf;
	// �ռ䲻�������ؿ�ָ��
	if (free_size < size) {
		buf = NULL;
	}
	else {
		// ��շ����ڴ�
		memset(buf, 0, size);
		// ���¿���ָ��λ��
		avs_buf->cur_pos = (void *)((QWORD)buf + size);
	}
	return buf;
}

float get_scale_value(	float	*data, 
						float	scale_i, 
						float	scale_j, 
						int		width, 
						int		height) {
	int pre_i = scale_i, pre_j = scale_j;
	float post_i = scale_i - pre_i, post_j = scale_j - pre_j;
	// �߽紦��
	if (pre_i + 1 >= width || pre_j + 1 >= height) {
		return data[pre_j * width + pre_i];
	}
	float x1 = data[pre_j * width + pre_i];
	float x2 = data[pre_j * width + pre_i + 1];
	float x3 = data[(pre_j + 1) * width + pre_i];
	float x4 = data[(pre_j + 1) * width + pre_i + 1];
	return (1 - post_i) * (1 - post_j) * x1 + post_i * post_j * x4 +
		(1 - post_j) * post_i * x2 + (1 - post_i) * post_j * x3;
}