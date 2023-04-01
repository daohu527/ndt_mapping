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

/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include "gflags/gflags.h"

#include "async_buffer.h"

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = pcl::PointCloud<Point>::Ptr;
using PointCloudConstPtr = const pcl::PointCloud<Point>::Ptr;

// filter
DEFINE_double(min_scan_range, 25.0, "the square of the min scan range");
DEFINE_double(max_scan_range, 10000.0, "the square of the max scan range");
DEFINE_double(min_add_scan_shift, 5.0, "the square of the min add scan length");
DEFINE_double(voxel_leaf_size, 2.0, "voxel leaf size");
// ndt
DEFINE_double(trans_eps, 0.01, "transformation epsilon");
DEFINE_double(step_size, 0.1, "step size");
DEFINE_double(ndt_res, 1.0, "ndt resolution");
DEFINE_int32(max_iter, 30, "maximum iterations times");
// map
DEFINE_string(output_file, "data/output.pcd", "map save file path");
DEFINE_string(workspace_dir, "data/pcd", "work dir");

// key pose
size_t CACHE_SIZE = 3;
std::list<PointCloudPtr> cache_key_frames;

// the whole map
PointCloudPtr global_map(new PointCloud());
PointCloudPtr local_map(new PointCloud());
bool is_first_map = true;

// runtime pose
Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
Eigen::Affine3d previous_pose = Eigen::Affine3d::Identity();
Eigen::Affine3d previous_key_pose = Eigen::Affine3d::Identity();

// lidar pose
std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> pcd_poses;
std::vector<double> time_stamps;
std::vector<unsigned int> pcd_indices;

// ndt
pcl::NormalDistributionsTransform<Point, Point> ndt;

//
std::unique_ptr<AsyncBuffer> async_buffer_ptr;


void Affine3dToPoint(const Eigen::Affine3d& affine3d_pose, Point* pose) {
  pose->x = affine3d_pose.translation().x();
  pose->y = affine3d_pose.translation().y();
  pose->z = affine3d_pose.translation().z();
}

void Init() {
  // ndt
  ndt.setTransformationEpsilon(FLAGS_trans_eps);
  ndt.setStepSize(FLAGS_step_size);
  ndt.setResolution(static_cast<float>(FLAGS_ndt_res));
  ndt.setMaximumIterations(FLAGS_max_iter);
}

void LoadPcdPoses(const std::string& file_path,
                  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>* poses,
                  std::vector<double>* timestamps,
                  std::vector<unsigned int>* pcd_indices) {
  poses->clear();
  timestamps->clear();
  pcd_indices->clear();

  FILE* file = fopen(file_path.c_str(), "r");
  if (file) {
    unsigned int index;
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qr;
    static constexpr int kSize = 9;
    while (fscanf(file, "%u %lf %lf %lf %lf %lf %lf %lf %lf\n", &index,
                  &timestamp, &x, &y, &z, &qx, &qy, &qz, &qr) == kSize) {
      Eigen::Translation3d trans(Eigen::Vector3d(x, y, z));
      Eigen::Quaterniond quat(qr, qx, qy, qz);
      poses->push_back(trans * quat);
      timestamps->push_back(timestamp);
      pcd_indices->push_back(index);
    }
    fclose(file);
  } else {
    std::cout << "Can't open file to read: " << file_path << std::endl;
  }
}

// load all poses from file
void LoadPoses() {
  std::string pose_file = FLAGS_workspace_dir + "/poses.txt";

  LoadPcdPoses(pose_file, &pcd_poses, &time_stamps, &pcd_indices);

  std::cout << "pcd_poses: " << pcd_poses.size()
        << " ,pcd_indices: " << pcd_indices.size() << std::endl;
}

void StartAsyncReadProcess() {
  std::vector<std::string> file_paths;
  for (unsigned int index : pcd_indices) {
    std::ostringstream ss;
    ss << FLAGS_workspace_dir << "/" << index << ".pcd";
    std::string pcd_file_path = ss.str();
    file_paths.push_back(pcd_file_path);
  }

  std::cout << "Pcd file size: " << file_paths.size() << std::endl;

  async_buffer_ptr.reset(new AsyncBuffer(file_paths));
  async_buffer_ptr->Init();
}

void RangeFilter(PointCloudConstPtr input, PointCloudPtr output) {
  for (PointCloud::const_iterator item = input->begin(); item != input->end();
        item++) {
    Point point;
    point.x = item->x;
    point.y = item->y;
    point.z = item->z;
    point.intensity = item->intensity;

    double r = std::pow(point.x, 2.0) + std::pow(point.y, 2.0);
    if (FLAGS_min_scan_range <= r && r <= FLAGS_max_scan_range) {
      output->push_back(point);
    }
  }
}

void VoxelFilter(PointCloudConstPtr input, PointCloudPtr output) {
  pcl::VoxelGrid<Point> voxel_grid_filter;
  float voxel_leaf_size = static_cast<float>(FLAGS_voxel_leaf_size);
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size,
      voxel_leaf_size);
  voxel_grid_filter.setInputCloud(input);
  voxel_grid_filter.filter(*output);
}

Eigen::Affine3d GetLidarRelativePose() {
  static unsigned int index = 1;
  Eigen::Affine3d relative_pose;
  if (index < pcd_poses.size()) {
    relative_pose = pcd_poses[index-1].inverse() * pcd_poses[index];
  }
  index++;
  return relative_pose;
}

double SquaredDistance(const Eigen::Affine3d& first,
                       const Eigen::Affine3d& second) {
  Eigen::Translation3d first_transd(first.translation());
  Eigen::Translation3d second_transd(second.translation());

  return std::pow(first_transd.x() - second_transd.x(), 2.0)
            + std::pow(first_transd.y() - second_transd.y(), 2.0);
}

void LidarProcess(PointCloudPtr cloud_ptr) {
  // CHECK(cloud_ptr != nullptr) << "cloud nullptr!";

  PointCloudPtr scan_ptr(new PointCloud());
  RangeFilter(cloud_ptr, scan_ptr);

  // When creating the map for the first time,
  // get the first frame and add it to the map, then return
  if (is_first_map) {
    *global_map += *scan_ptr;
    *local_map += *scan_ptr;
    cache_key_frames.push_back(scan_ptr);
    is_first_map = false;
    return;
  }

  PointCloudPtr voxel_ptr(new PointCloud());
  VoxelFilter(scan_ptr, voxel_ptr);

  ndt.setInputSource(voxel_ptr);
  ndt.setInputTarget(local_map);

  // Get the relative pose between 2 frames
  Eigen::Affine3d relative_pose = GetLidarRelativePose();
  // Calculate the guess pose based on the position of the previous frame
  Eigen::Matrix4f guess_pose =
      (previous_pose * relative_pose).matrix().cast<float>();

  PointCloudPtr output_cloud(new PointCloud());
  ndt.align(*output_cloud, guess_pose);

  double fitness_score = ndt.getFitnessScore();
  Eigen::Matrix4f t_localizer = ndt.getFinalTransformation();
  bool has_converged = ndt.hasConverged();
  int final_num_iteration = ndt.getFinalNumIteration();
  double transformation_probability = ndt.getTransformationProbability();

  static int sequence_id = 0;
  sequence_id++;
  std::cout << "NDT sequence id: " << sequence_id
        << " fitness_score: " << fitness_score
        << " has_converged: " << has_converged
        << " final_num_iteration: " << final_num_iteration
        << " transformation_probability: " << transformation_probability
        << std::endl;

  current_pose = t_localizer.cast<double>();
  previous_pose = current_pose;

  double shift = SquaredDistance(current_pose, previous_key_pose);
  if (shift >= FLAGS_min_add_scan_shift) {
    PointCloudPtr transformed_scan_ptr(new PointCloud());
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
    *global_map += *transformed_scan_ptr;

    if (cache_key_frames.size() > CACHE_SIZE) {
      cache_key_frames.pop_front();
    }
    cache_key_frames.push_back(transformed_scan_ptr);

    local_map->clear();
    for (auto key_frame : cache_key_frames) {
      *local_map += *key_frame;
    }

    previous_key_pose = current_pose;
  }

  // Todo(zero): Add for online mode
  // Publish map
  // Publish pose
}

void SaveMap() {
  // Align coordinates.
  // The initial coordinates are the pose of the first frame
  // CHECK(pcd_poses.size() != 0) << "pcd pose is empty";
  Eigen::Affine3d init_pose = pcd_poses[0];
  Eigen::Affine3f align_pose = Eigen::Affine3f::Identity();
  align_pose.linear() = init_pose.cast<float>().linear();

  std::cout << "Align matrix: " << align_pose.matrix() << std::endl;

  // Quaterniond
  Eigen::Quaterniond quaterniond = (Eigen::Quaterniond)init_pose.linear();
  std::cout << "Align quaterniond x: " << quaterniond.x()
        << " y: " << quaterniond.y()
        << " z: " << quaterniond.z()
        << " w: " << quaterniond.w()
        << std::endl;

  // Rotation
  auto euler = quaterniond.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
  std::cout << "Align rotation roll, pitch, yaw " << euler << std::endl;

  // CHECK(global_map != nullptr) << "map is null";
  PointCloudPtr align_map_ptr(new PointCloud());
  pcl::transformPointCloud(*global_map, *align_map_ptr, align_pose.matrix());

  std::cout << std::setprecision(15) << "UTM relative coordinates"
        << " x: " << init_pose.translation().x()
        << " y: " << init_pose.translation().y()
        << " z: " << init_pose.translation().z() << std::endl;

  // Save map
  pcl::io::savePCDFileBinaryCompressed(FLAGS_output_file, *align_map_ptr);
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  auto start_time = std::chrono::system_clock::now();

  Init();
  LoadPoses();

  StartAsyncReadProcess();

  while(!async_buffer_ptr->IsEnd()) {
    PointCloudPtr cloud_ptr = async_buffer_ptr->Get();
    if (cloud_ptr) {
      LidarProcess(cloud_ptr);
    }
  }

  SaveMap();

  // Performance
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  std::cout << "NDT mapping cost:" << diff.count() << " s" << std::endl;
}
