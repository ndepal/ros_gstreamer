/****************************************************************************
*
*   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
*
* Redistribution and use in stream_source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of stream_source code must retain the above copyright
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
* ROSGStreamer.cpp
*
*  Created on: Mar 21, 2016
*      Author: nicolas
*/

#include "ROSGStreamer.h"
#include <gst/gst.h>

ROSGStreamer::ROSGStreamer()
: nh("~"),
initialized(false) {

	std::string img_topic, stream_IP;
	int stream_FPS, stream_res_w, stream_port;
	if (!nh.getParam("image_topic", img_topic)) {
		img_topic = "/snap_cam/highres/image";
		ROS_WARN("No image topic provided. Defaulting to %s", img_topic.c_str());
	}
	if (!nh.getParam("stream_FPS", stream_FPS)) {
		stream_FPS = 30;
		ROS_WARN("No streaming framerate provided. Defaulting to %d", stream_FPS);
	}
	if (!nh.getParam("stream_resolution_width", stream_res_w)) {
		stream_res_w = 0;
		ROS_WARN("No streaming resolution width provided. Streaming in original resolution");
	}
	if (!nh.getParam("stream_IP", stream_IP)) {
		stream_IP = "127.0.0.1";
		ROS_WARN("No streaming IP provided. Defaulting to %s", stream_IP.c_str());
	}
	if (!nh.getParam("stream_port", stream_port)) {
		stream_port = 5000;
		ROS_WARN("No streaming port provided. Defaulting to %d", stream_port);
	}

    /* Initialize GStreamer */
    gst_init (0, 0);

	/* set up the streaming pipeline */
	std::stringstream stream_stream_pipeline_ss;
	stream_stream_pipeline_ss << "appsrc name=source ! videoconvert ! "
				<< "videorate ! video/x-raw, framerate=(fraction)" << stream_FPS << "/1 ! ";
	if (stream_res_w)
	{
		stream_stream_pipeline_ss << "videoscale ! video/x-raw,width=" << stream_res_w << " ! ";
	}
	stream_stream_pipeline_ss << "x264enc speed-preset=ultrafast tune=zerolatency ! "
				<< "video/x-h264, stream-format=(string)avc, alignment=(string)au, level=(string)2, profile=(string)high, framerate=(fraction)" << stream_FPS << "/1 ! "
				<< "rtph264pay ! "
				<< "udpsink host=" << stream_IP << " port=" << stream_port;

	ROS_INFO("Streaming pipeline: %s", stream_stream_pipeline_ss.str().c_str());

    GError *error = nullptr;
	stream_pipeline = gst_parse_launch(stream_stream_pipeline_ss.str().c_str(), &error);
	if (error != nullptr) {
		ROS_ERROR("GST: %s", error->message);
		g_error_free(error);
		return;
	} else {
		ROS_INFO("gst_parse_launch successful");
	}

	img_sub = nh.subscribe(img_topic.c_str(), 1, &ROSGStreamer::imgCb, this);
    ROS_INFO("End constructor");
}

ROSGStreamer::~ROSGStreamer() {
    /* Free resources */
    //   gst_object_unref (bus);
    gst_element_set_state (stream_pipeline, GST_STATE_NULL);
    gst_object_unref (stream_pipeline);
}

void ROSGStreamer::imgCb(const sensor_msgs::Image &msg)
{
    // last_image = msg;
    // return;
    // ROS_INFO("img cb %d x %d, %s", msg.width, msg.height, msg.encoding.c_str());

	cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, msg.encoding);

	if (!initialized)
	{
		init(img->image.rows, img->image.cols);
		ROS_INFO("Image encoding: %s", msg.encoding.c_str());
	}

	cv::Mat img_color;
	if (msg.encoding == "rgb8")
	{
		// cv::cvtColor(img->image, img_color, CV_BGR2RGB);
		img_color = img->image;
	} else {
		cv::cvtColor(img->image, img_color, CV_GRAY2RGB);
	}

	int bufferlength = img_color.cols * img_color.rows * img_color.channels();
    GstBuffer *buffer = gst_buffer_new_and_alloc( bufferlength );

	gst_buffer_fill(buffer, 0, img_color.data, bufferlength);
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);

	auto push_ret = gst_app_src_push_buffer(GST_APP_SRC_CAST(stream_source), buffer);

	return;
}

void ROSGStreamer::init(int height, int width) {
	ROS_INFO("Initializing with source video resolution %d x %d", width, height);

	stream_source = gst_bin_get_by_name(GST_BIN(stream_pipeline), "source");
	g_object_set (G_OBJECT (stream_source), "caps",
	gst_caps_new_simple ("video/x-raw",
	"format", G_TYPE_STRING, "RGB",
	"width", G_TYPE_INT, width,
	"height", G_TYPE_INT, height,
	"framerate", GST_TYPE_FRACTION, 0, 1,
	NULL), NULL);
	gst_app_src_set_stream_type(GST_APP_SRC_CAST(stream_source), GST_APP_STREAM_TYPE_STREAM);
	gst_app_src_set_latency(GST_APP_SRC_CAST(stream_source), 0, -1);
	g_object_set (G_OBJECT (stream_source),
	"format", GST_FORMAT_TIME,
	"is-live", true,
	NULL);

	/* Start playing */
    ret = gst_element_set_state (stream_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the stream_pipeline to the playing state.\n");
        gst_object_unref (stream_pipeline);
        return;
    }

	initialized = true;
}
