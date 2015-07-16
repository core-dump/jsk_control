// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef JSK_FOOTSTEP_PLANNER_FOOTSTEP_STATE_H_
#define JSK_FOOTSTEP_PLANNER_FOOTSTEP_STATE_H_

#include <jsk_footstep_msgs/Footstep.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/octree.h>

namespace jsk_footstep_planner
{

  namespace projection_state
  {
    const unsigned int success = 1;
    const unsigned int no_pointcloud = 2;
    const unsigned int no_enough_support = 4;
    const unsigned int no_plane = 8;
    const unsigned int no_enough_inliers = 16;
  }
  
  class FootstepState
  {
  public:
    typedef boost::shared_ptr<FootstepState> Ptr;
    FootstepState(int leg,
                  const Eigen::Affine3f& pose,
                  const Eigen::Vector3f& dimensions,
                  const Eigen::Vector3f& resolution):
      leg_(leg), pose_(pose), dimensions_(dimensions), resolution_(resolution)
      {
        float x = pose_.translation()[0];
        float y = pose_.translation()[1];
        float roll, pitch, yaw;
        pcl::getEulerAngles(pose_, roll, pitch, yaw);
        index_x_ = x / resolution_[0];
        index_y_ = y / resolution_[1];
        index_yaw_ = yaw / resolution_[2];
      }

    FootstepState(int leg,
                  const Eigen::Affine3f& pose,
                  const Eigen::Vector3f& dimensions,
                  const Eigen::Vector3f& resolution,
                  int index_x,
                  int index_y,
                  int index_yaw):
      leg_(leg), pose_(pose), dimensions_(dimensions), resolution_(resolution),
      index_x_(index_x), index_y_(index_y), index_yaw_(index_yaw)
      {
      }
    
    inline float cross2d(const Eigen::Vector2f& a, const Eigen::Vector2f& b)
    {
      return a[0] * b[1] - a[1] * b[0];
    }
    virtual jsk_footstep_msgs::Footstep::Ptr toROSMsg();
    virtual FootstepState::Ptr
    projectToCloud(pcl::KdTreeFLANN<pcl::PointNormal>& tree,
                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                   pcl::search::Octree<pcl::PointNormal>& tree_2d,
                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_2d,
                   const Eigen::Vector3f& z,
                   unsigned int& error_state,
                   double outlier_threshold,
                   int max_iterations,
                   int min_inliers);
    pcl::PointIndices::Ptr
    cropPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                   pcl::search::Octree<pcl::PointNormal>& tree);
        
    virtual Eigen::Affine3f getPose() { return pose_; }
    virtual void setPose(const Eigen::Affine3f& pose) { pose_ = pose; }
    virtual int getLeg() { return leg_; }
    virtual Eigen::Vector3f getDimensions() { return dimensions_; }
    bool operator==(FootstepState& other)
    {
      return ((index_x_ == other.index_x_) &&
              (index_y_ == other.index_y_) &&
              (index_yaw_ == other.index_yaw_));
    }

    inline virtual int indexX() { return index_x_; }
    inline virtual int indexY() { return index_y_; }
    inline virtual int indexT() { return index_yaw_; }
    
  protected:
    Eigen::Affine3f pose_;
    const Eigen::Vector3f dimensions_;
    const Eigen::Vector3f resolution_; // not memory efficient?
    const int leg_;
    int index_x_;
    int index_y_;
    int index_yaw_;
  private:
    
  };

  inline size_t hash_value(const FootstepState::Ptr& s)
  {
    return std::abs(s->indexX()) << (25 + 14) + std::abs(s->indexY()) << 14
      + std::abs(s->indexT());
  }
  
}

#endif
