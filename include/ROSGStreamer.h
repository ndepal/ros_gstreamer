/****************************************************************************
 *
 *   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name AIT nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * ROSGStreamer.h
 *
 *  Created on: Mar 21, 2016
 *      Author: nicolas
 */
#ifndef ROS_GSTREAMER
#define ROS_GSTREAMER

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <gst/gst.h>
#include <gst/app/app.h>
// #include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <iterator>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sstream>

class ROSGStreamer {
public:
    ROSGStreamer();
    ~ROSGStreamer();

    sensor_msgs::Image last_image;

    void init_recording();
    void stop_recording();

private:
    GstElement *stream_pipeline, *stream_source;
    GstElement *record_pipeline, *record_source;
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;

    bool stream_initialized;
    void init_stream();
    int original_width, original_height;

    bool recording, record_always;
    std::string out_file;
    int record_FPS, record_res_w;
    ros::Subscriber record_sub;
    void recordCb(const std_msgs::Bool &msg);

    ros::NodeHandle nh;

    ros::Subscriber img_sub;
    void imgCb(const sensor_msgs::Image &msg);
    GstSample* gst_sample_new_from_image(const sensor_msgs::Image &msg);
    GstCaps* gst_caps_new_from_image(const sensor_msgs::Image &msg);

    void determineFilePath(const std::string &path, std::string &unique_path);
};
#endif
