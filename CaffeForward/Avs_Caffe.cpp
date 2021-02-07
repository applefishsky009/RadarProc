#include "Avs_Radar.h"
#include "Avs_Caffe.h"
#include "Avs_Caffe_lib.h"
#include "Avs_Caffe_Common_lib.h"
#include "caffe/caffe.hpp"
#include "caffe/caffe_reg.hpp"

unsigned int AVS_Caffe_GetMemSize(	AVS_CAFFE_INFO			*input,
									AVS_MEM_TAB				mem_tab[AVS_RADAR_MEM_TAB]) {
	unsigned int mem_size = 0;

	AVS_CHECK_ERROR(input->proto == NULL, AVS_LIB_KEY_PARAM_ERR);
	AVS_CHECK_ERROR(input->model == NULL, AVS_LIB_KEY_PARAM_ERR);

	// 算法库句柄内存
	mem_size += sizeof(AVS_CAFFE_FILTER) + AVS_MEM_ALIGN_128BYTE;

	mem_tab[0].size = mem_size;
	mem_tab[0].base = NULL;
	mem_tab[0].alignment = AVS_MEM_ALIGN_128BYTE;
	return 0;
}

unsigned int AVS_Caffe_CreatMemSize(	AVS_CAFFE_INFO			*input,
										AVS_MEM_TAB				mem_tab[AVS_RADAR_MEM_TAB],
										void					**handle) {
	int i = 0;
	AVS_CAFFE_FILTER *filter = NULL;
	AVS_CAFFE_BUF mem_buf;
	mem_buf.start = mem_tab[0].base;
	mem_buf.cur_pos = mem_tab[0].base;
	mem_buf.end = (void *)((QWORD)mem_buf.cur_pos + (QWORD)mem_tab[0].size);

	AVS_CHECK_ERROR(input->proto == NULL, AVS_LIB_KEY_PARAM_ERR);
	AVS_CHECK_ERROR(input->model == NULL, AVS_LIB_KEY_PARAM_ERR);

	// 算法库句柄内存
	filter = (AVS_CAFFE_FILTER *)AVS_CAFFE_COM_alloc_buffer(&mem_buf, sizeof(AVS_CAFFE_FILTER));

	*handle = (void *)filter;
	return 0;
}

unsigned int AVS_Caffe_Init(void						*handle, 
							AVS_CAFFE_INFO				*inbuf,
							int							in_buf_size) {
	AVS_CAFFE_FILTER *caffe_filter = (AVS_CAFFE_FILTER *)handle;
	// 初始化网络
	Phase phase = TEST;
	//Caffe::set_mode(Caffe::CPU);
	Caffe::set_mode(Caffe::GPU);
	caffe_filter->net = boost::shared_ptr<Net<float>> (new caffe::Net<float>(inbuf->proto, phase));
	// 加载已经训练好的模型
	caffe_filter->net->CopyTrainedLayersFrom(inbuf->model);
	return 0;
}

unsigned int AVS_Caffe_Forward(	void						*handle,
								AVS_CAFFE_INFO				*inbuf,
								int							in_buf_size,
								AVS_RADAR_POSTPROCESS_IN	*outbuf,
								int							out_buf_size) {
	int idx = 0;
	AVS_CAFFE_FILTER *caffe_filter = (AVS_CAFFE_FILTER *)handle;

	// 网络前向
	boost::shared_ptr<Net<float>> net = caffe_filter->net;
	caffe_forward(net, inbuf->caffe_input.data);

	// 读取回归信息
	char *reg = "regression";
	idx = get_blob_index(net, reg);
	boost::shared_ptr<Blob<float>> blobs_reg = net->blobs()[idx];
	unsigned int num_data_reg = blobs_reg->count();
	const float *blob_ptr_reg = (const float *)blobs_reg->cpu_data();
	for (int i = 0, j = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		outbuf->regression[i]->x1 = *(blob_ptr_reg + j++);
		outbuf->regression[i]->y1 = *(blob_ptr_reg + j++);
		outbuf->regression[i]->x2 = *(blob_ptr_reg + j++);
		outbuf->regression[i]->y2 = *(blob_ptr_reg + j++);
	}

	// 读取分类信息
	char *cls = "classification";
	idx = get_blob_index(net, cls);
	boost::shared_ptr<Blob<float>> blobs_cls = net->blobs()[idx];
	unsigned int num_data_cls = blobs_cls->count();
	const float *blob_ptr_cls = (const float *)blobs_cls->cpu_data();
	for (int i = 0, j = 0; i < AVS_POST_OUT_LENGTH; ++i) {
		outbuf->classification[i]->conf[0] = *(blob_ptr_cls + j++);
		outbuf->classification[i]->conf[1] = *(blob_ptr_cls + j++);
		outbuf->classification[i]->conf[2] = *(blob_ptr_cls + j++);
		outbuf->classification[i]->conf[3] = *(blob_ptr_cls + j++);
		outbuf->classification[i]->conf[4] = *(blob_ptr_cls + j++);
		outbuf->classification[i]->conf[5] = *(blob_ptr_cls + j++);
		outbuf->classification[i]->conf[6] = *(blob_ptr_cls + j++);
		outbuf->classification[i]->conf[7] = *(blob_ptr_cls + j++);
	}
	return 0;
}