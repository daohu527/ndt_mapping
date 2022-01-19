/*
 * Copyright (c) 2021 daohu527 <daohu527@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/


#include <thread>
#include <chrono>

#include "async_buffer.h"

PointCloudPtr AsyncBuffer::Get() {
  PointCloudPtr cloud_ptr = nullptr;

  if (Empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!Empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_ptr = buffer_.front();
    buffer_.pop();
  }

  return cloud_ptr;
}

void AsyncBuffer::Init() {
  std::thread async_read_process([this] {
    while(index_ < total_size_) {
      if (Size() < BUFFER_SIZE) {
        std::string pcd_file_path = file_paths_[index_];
        PointCloudPtr cloud_ptr(new PointCloud());
        pcl::io::loadPCDFile(pcd_file_path, *cloud_ptr);
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.push(cloud_ptr);
        index_++;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });

  async_read_process.detach();
}

bool AsyncBuffer::IsEnd() {
  std::lock_guard<std::mutex> lock(mutex_);
  return (index_ >= total_size_) && buffer_.empty();
}

std::size_t AsyncBuffer::Size() {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.size();
}

bool AsyncBuffer::Empty() {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.empty();
}
