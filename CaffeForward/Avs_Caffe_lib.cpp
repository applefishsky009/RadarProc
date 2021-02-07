#include "Avs_Caffe_lib.h"

// 根据blob_name获取其在网络中的index
unsigned int get_blob_index(boost::shared_ptr<Net<float>> &net, 
							char *query_blob_name) {
	std::string str_query(query_blob_name);
	vector<string> const &blob_names = net->blob_names();
	for (int i = 0; i < blob_names.size(); ++i) {
		if (str_query == blob_names[i]) {
			return i;
		}
	}
	LOG(FATAL) << "Unknown blob name: " << str_query;
}

// 根据layer名字获取其在网络中的Index
unsigned int get_layer_index(	boost::shared_ptr<Net<float>> &net, 
								char *query_layer_name) {
	std::string str_query(query_layer_name);
	vector<string> const &layer_names = net->layer_names();
	for (unsigned int i = 0; i < layer_names.size(); ++i) {
		if (str_query == layer_names[i]) {
			return i;
		}
	}
	LOG(FATAL) << "Unknown layer name:" << str_query;
}

// 前向传播
void caffe_forward(	boost::shared_ptr<Net<float>> &net, 
					float *data_ptr) {
	Blob<float>* input_blobs = net->input_blobs()[0];
	switch (Caffe::mode()) {
	case Caffe::CPU:
		memcpy(input_blobs->mutable_cpu_data(), data_ptr, 
			sizeof(float) * input_blobs->count());
		break;
	case Caffe::GPU:
		cudaMemcpy(input_blobs->mutable_gpu_data(), data_ptr,
			sizeof(float) * input_blobs->count(), cudaMemcpyHostToDevice);
		break;
	default:
		LOG(FATAL) << "Unknown Caffe mode";
	}
	net->ForwardPrefilled();
}