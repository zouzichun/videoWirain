#include "img_process.h"
#include "crcalgorithm.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "spdlog/spdlog.h"
#include "maindialog.h"
#include <QByteArray>
#include <QDebug>
#include <QPainter>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace std;
using namespace cv;

#include "RgaUtils.h"
#include "rknn_api.h"

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
  std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
  for (int i = 1; i < attr->n_dims; ++i)
  {
    shape_str += ", " + std::to_string(attr->dims[i]);
  }

  printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride = %d, size_with_stride=%d, fmt=%s, "
         "type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, shape_str.c_str(), attr->n_elems, attr->size, attr->w_stride,
         attr->size_with_stride, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
  unsigned char *data;
  int ret;

  data = NULL;

  if (NULL == fp)
  {
    return NULL;
  }

  ret = fseek(fp, ofst, SEEK_SET);
  if (ret != 0)
  {
    printf("blob seek failure.\n");
    return NULL;
  }

  data = (unsigned char *)malloc(sz);
  if (data == NULL)
  {
    printf("buffer malloc failure.\n");
    return NULL;
  }
  ret = fread(data, 1, sz, fp);
  return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{
  FILE *fp;
  unsigned char *data;

  fp = fopen(filename, "rb");
  if (NULL == fp)
  {
    printf("Open file %s failed.\n", filename);
    return NULL;
  }

  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);

  data = load_data(fp, 0, size);

  fclose(fp);

  *model_size = size;
  return data;
}

ImgProcessRknn::ImgProcessRknn(QString model_name) : ImgProcess(model_name, 2048, 2048, true) {
}

ImgProcessRknn::~ImgProcessRknn() {
}

bool ImgProcessRknn::Init() {
    int ret = 0;
    /* Create the neural network */
    spdlog::info("Loading model: {}", m_name.toStdString());
    int model_data_size = 0;
    unsigned char *model_data = load_model(m_name.toStdString().c_str(), &model_data_size);
    ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0)
    {
      printf("rknn_init error ret=%d\n", ret);
      return false;
    }

    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0)
    {
    printf("rknn_init error ret=%d\n", ret);
    return false;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0)
    {
    printf("rknn_init error ret=%d\n", ret);
    return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
    input_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    dump_tensor_attr(&(input_attrs[i]));
    }

    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
    output_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
    dump_tensor_attr(&(output_attrs[i]));
    }

    int channel = 3;
    int width = 0;
    int height = 0;
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
    printf("model is NCHW input fmt\n");
    channel = input_attrs[0].dims[1];
    height = input_attrs[0].dims[2];
    width = input_attrs[0].dims[3];
    }
    else
    {
    printf("model is NHWC input fmt\n");
    height = input_attrs[0].dims[1];
    width = input_attrs[0].dims[2];
    channel = input_attrs[0].dims[3];
    }

    printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);
}
