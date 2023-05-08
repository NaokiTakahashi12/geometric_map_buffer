// MIT License
//
// Copyright (c) 2023 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <geometric_map_buffer/grid_map_converter.hpp>

#include <string>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <grid_map_cv/grid_map_cv.hpp>


namespace geometric_map_buffer::grid_map_converter
{
template<typename Type, int NChannels>
void addLayerFromImageUseParam(
  grid_map::GridMap & grid_map,
  const cv::Mat & cv_mat,
  const BuildGridMapLayerParameters & param
)
{
  grid_map::GridMapCvConverter::addLayerFromImage<Type, NChannels>(
    cv_mat,
    param.layer_name,
    grid_map,
    param.lower_value,
    param.upper_value,
    param.alpha_threshold
  );
}

BuildGridMapLayerParameters::BuildGridMapLayerParameters()
: layer_name("elevation"),
  lower_value(0.0),
  upper_value(1.0),
  alpha_threshold(0.5)
{
}

void addLayerFromImage(
  grid_map::GridMap & grid_map,
  const cv::Mat & cv_mat,
  const BuildGridMapLayerParameters & param
)
{
  const int cv_mat_encoding = cv_mat.type() & CV_MAT_DEPTH_MASK;
  switch (cv_mat_encoding) {
    case CV_8UC1:
      addLayerFromImageUseParam<uint8_t, 1>(grid_map, cv_mat, param);
      break;
    case CV_8UC3:
      addLayerFromImageUseParam<uint8_t, 3>(grid_map, cv_mat, param);
      break;
    case CV_8UC4:
      addLayerFromImageUseParam<uint8_t, 4>(grid_map, cv_mat, param);
      break;
    case CV_16UC1:
      addLayerFromImageUseParam<uint16_t, 1>(grid_map, cv_mat, param);
      break;
    case CV_16UC3:
      addLayerFromImageUseParam<uint16_t, 3>(grid_map, cv_mat, param);
      break;
    case CV_16UC4:
      addLayerFromImageUseParam<uint16_t, 4>(grid_map, cv_mat, param);
      break;
    default:
      throw std::runtime_error("Expected image encoding");
  }
}
}  // namespace geometric_map_buffer::grid_map_converter
