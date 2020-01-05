/*
 * MIT License
 * 
 * Copyright (c) 2020 Koki Shinjo
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

// Standard C++ Library
#include <queue>
// Boost
#include <boost/thread.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pointcloud_accumulator/pointcloud_accumulator.h>

namespace pointcloud_accumulator {

PointCloudAccumulator::PointCloudAccumulator(ros::NodeHandle &nh, ros::NodeHandle &nh_private, tf2_ros::Buffer &tf_buffer)
    :mode_callback_(NONE), duration_callback_(-1)
{
    // ROS params
    nh_private.param<std::string>("fixed_frame", fixed_frame_id_, "map");
    nh_private.param<double>("max_duration", max_duration_, 5.0);
    nh_private.param<double>("publish_rate", publish_rate_, 1.0);
    nh_private.param<int>("num_spinthread", num_spinthread_, 5);

    //
    sub_ = nh.subscribe<sensor_msgs::PointCloud2>( "input", 10, boost::bind(&PointCloudAccumulator::callback, this, boost::ref(tf_buffer), _1) );
    pub_ = nh.advertise<sensor_msgs::PointCloud2>( "output", 1 );

    // add spinner
    ptr_spinner_ = boost::shared_ptr<ros::AsyncSpinner>( new ros::AsyncSpinner(num_spinthread_) );
    ptr_spinner_->start();

    // measure callback hz
    this->measureCallbackRate();
}

void PointCloudAccumulator::measureCallbackRate()
{
    //
    sample_callbacked_ = 0;
    mode_callback_ = MEASURING;
    ros::Rate loop_rate(publish_rate_);
    time_callbacked_ = ros::Time::now();
    //
    loop_rate.sleep();
    mode_callback_ = NONE;
    //
    // TODO:ゼロ除算時のエラーをthrowする
    duration_callback_ = ( ros::Time::now() - time_callbacked_ ).toSec() / sample_callbacked_;
    max_size_ = (int)( max_duration_ / duration_callback_ );
    //
    ROS_INFO("message duration: %lf", duration_callback_);
}

void PointCloudAccumulator::spin()
{
    mode_callback_ = STORING;
    ros::Rate loop_rate(publish_rate_);

    while (ros::ok()) {
        this->publish();
        loop_rate.sleep();
    }

    mode_callback_ = NONE;
    ptr_spinner_->stop();
}


void PointCloudAccumulator::publish()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 pubmsg;

    mtx_vector_.lock();
    for ( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator pclptr_itr = vector_pointcloud_.begin();
            pclptr_itr != vector_pointcloud_.end();
            pclptr_itr++ ) {
        *cloud_pub += *(*pclptr_itr);
    }
    mtx_vector_.unlock();

    ROS_INFO("publish a message. vector length:%lu, pc size:%lu", vector_pointcloud_.size(), cloud_pub->points.size());

    pcl::toROSMsg( *cloud_pub, pubmsg );
    pubmsg.header.frame_id = fixed_frame_id_;
    pubmsg.header.stamp = ros::Time::now();
    pub_.publish( pubmsg );
}


void PointCloudAccumulator::callback( tf2_ros::Buffer &tf_buffer, 
                                      const sensor_msgs::PointCloud2::ConstPtr &msgptr )
{
    //
    switch ( mode_callback_) {
        case NONE:
            ROS_INFO("callback called, do nothing.");
            break;

        case STORING:
            ROS_INFO("callback called, storing.");
            try {
                //
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);
                //
                geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(
                                                              fixed_frame_id_,
                                                              msgptr->header.stamp,
                                                              msgptr->header.frame_id,
                                                              msgptr->header.stamp,
                                                              fixed_frame_id_ );
                Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
                // convert
                pcl::fromROSMsg(*msgptr, *cloud_src);
                // transform
                pcl::transformPointCloud(*cloud_src, *cloud_dst, mat);
                //
                mtx_vector_.lock();
                vector_pointcloud_.push_back(cloud_dst);
                while ( vector_pointcloud_.size() > max_size_ ) {
                    vector_pointcloud_.erase(vector_pointcloud_.begin());
                }
                mtx_vector_.unlock();
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
            }
            break;

        case MEASURING:
            ROS_INFO("callback called, measuring,");
            sample_callbacked_++;
            break;

        default:
            ROS_ERROR("Unknown callback mode is set.");
    }
}

}
