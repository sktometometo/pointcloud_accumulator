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

#ifndef POINTCLOUD_ACCUMULATOR_POINTCLOUD_ACCUMULATOR_H
#define POINTCLOUD_ACCUMULATOR_POINTCLOUD_ACCUMULATOR_H

// Standard C++ Library
#include <iostream>
#include <queue>
// PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace pointcloud_accumulator {

    enum CallbackMode {
        NONE, STORING, MEASURING
    };

    class PointCloudAccumulator
    {
        public:
            PointCloudAccumulator(ros::NodeHandle&, ros::NodeHandle&, tf2_ros::Buffer&);
            void spin();
            void measureCallbackRate();

        private:
            //
            void callback( tf2_ros::Buffer&, const sensor_msgs::PointCloud2::ConstPtr& );
            void publish();
            //
            ros::Subscriber sub_;
            ros::Publisher  pub_;
            // ros param values
            std::string     fixed_frame_id_;
            double          max_duration_;
            double          publish_rate_;
            int             num_spinthread_;
            //
            ros::Time       time_callbacked_;
            int             sample_callbacked_;
            CallbackMode    mode_callback_;
            double          duration_callback_;
            //
            boost::shared_ptr<ros::AsyncSpinner> ptr_spinner_;
            boost::mutex mtx_vector_;
            //
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vector_pointcloud_;
            int max_size_;
    };
}

#endif
