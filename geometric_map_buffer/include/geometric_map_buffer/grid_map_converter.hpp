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

#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include <grid_map_core/grid_map_core.hpp>

#include "geometric_map_buffer.hpp"


namespace geometric_map_buffer::grid_map_converter
{
struct BuildGridMapLayerParameters
{
  using UniquePtr = std::unique_ptr<BuildGridMapLayerParameters>;
  using SharedPtr = std::shared_ptr<BuildGridMapLayerParameters>;

  explicit BuildGridMapLayerParameters();

  std::string layer_name;
  float lower_value;
  float upper_value;
  double alpha_threshold;
};

void addLayerFromImage(
  ::grid_map::GridMap &,
  const cv::Mat &,
  const BuildGridMapLayerParameters &
);
}  // namespace geometric_map_buffer::grid_map_converter
