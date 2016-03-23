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
#include <unistd.h>
#include <fstream>

// helper functions for file IO
inline bool file_exists(const std::string& file) {
	printf("checking %s for existence\n", file.c_str());
    std::ifstream f(file.c_str());
    if (f.good()) {
        f.close();
        return true;
    } else {
        f.close();
		return false;
    }
}

// separate file_name at the extension. Name is the file name without extension, ext is the extension
// if file_name has no extension, the function returns false, otherwise true
inline bool separate_extension(const std::string& file_name, std::string& name, std::string& ext) {
	size_t separator_idx = file_name.find_last_of(".");
	name = file_name.substr(0, separator_idx);
	ext = file_name.substr(separator_idx+1);
	return separator_idx != file_name.size();
}

ROSGStreamer::ROSGStreamer()
: nh("~"),
stream_initialized(false),
recording(false) {

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

	if (!nh.getParam("record_FPS", record_FPS)) {
		record_FPS = stream_FPS;
		ROS_WARN("No recording framerate provided. Defaulting to same as streaming");
	}
	if (!nh.getParam("record_resolution_width", stream_res_w)) {
		record_res_w = 0;
		ROS_WARN("No recording resolution width provided. Recording in original resolution");
	}
	if (!nh.getParam("record_filename", out_file)) {
		out_file = "recording.mkv";
		ROS_WARN("No output file name provided. Defaulting to %s", out_file.c_str());
	}
	if (!nh.getParam("record_always", record_always)) {
		record_always = false;
	}

    /* Initialize GStreamer */
    gst_init (0, 0);

	/* set up the streaming pipeline */
	std::stringstream stream_pipeline_ss;
	stream_pipeline_ss << "appsrc name=source ! videoconvert ! "
				<< "videorate ! video/x-raw, framerate=(fraction)" << stream_FPS << "/1 ! ";
	// if (stream_res_w)
	// {
	// 	stream_pipeline_ss << "videoscale ! video/x-raw,width=" << stream_res_w << " ! ";
	// }
	// TODO: can qgc only accept 320x240?
	stream_pipeline_ss << "videoscale ! video/x-raw,width=320,height=240 ! ";

	stream_pipeline_ss << "x264enc speed-preset=ultrafast tune=zerolatency ! "
				<< "video/x-h264, stream-format=(string)avc, alignment=(string)au, level=(string)2, profile=(string)high, framerate=(fraction)" << stream_FPS << "/1 ! "
				<< "rtph264pay ! "
				<< "udpsink host=" << stream_IP << " port=" << stream_port;

	ROS_INFO("Streaming pipeline: %s", stream_pipeline_ss.str().c_str());

    GError *error = nullptr;
	stream_pipeline = gst_parse_launch(stream_pipeline_ss.str().c_str(), &error);
	if (error != nullptr) {
		ROS_ERROR("GST: %s", error->message);
		g_error_free(error);
		return;
	} else {
		ROS_INFO("gst_parse_launch successful for streaming pipeline");
	}

	img_sub = nh.subscribe(img_topic.c_str(), 1, &ROSGStreamer::imgCb, this);
	record_sub = nh.subscribe("record", 1, &ROSGStreamer::recordCb, this);
}

ROSGStreamer::~ROSGStreamer() {
    /* Free resources */
    gst_element_set_state (stream_pipeline, GST_STATE_NULL);
    gst_object_unref (stream_pipeline);

    stop_recording();
}

void ROSGStreamer::imgCb(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, msg.encoding);

	if (!stream_initialized)
	{
		original_width = img->image.cols;
		original_height = img->image.rows;
		init_stream();
		ROS_INFO("Image encoding: %s", msg.encoding.c_str());
	}

	if (!recording && record_always)
	{
		init_recording();
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
	if (push_ret != GST_FLOW_OK)
		ROS_WARN("Pushing to streaming pipeline was not successful");

	if (recording){
		buffer = gst_buffer_new_and_alloc( bufferlength );
		gst_buffer_fill(buffer, 0, img_color.data, bufferlength);
	    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);
		GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);
		push_ret = gst_app_src_push_buffer(GST_APP_SRC_CAST(record_source), buffer);
		if (push_ret != GST_FLOW_OK)
			ROS_WARN("Pushing to recording pipeline was not successful");
	}

	return;
}

void ROSGStreamer::init_stream() {
	ROS_INFO("Initializing stream with source video resolution %d x %d", original_width, original_height);

	stream_source = gst_bin_get_by_name(GST_BIN(stream_pipeline), "source");
	g_object_set (G_OBJECT (stream_source), "caps",
		gst_caps_new_simple ("video/x-raw",
			"format", G_TYPE_STRING, "RGB",
			"width", G_TYPE_INT, original_width,
			"height", G_TYPE_INT, original_height,
			"framerate", GST_TYPE_FRACTION, 0, 1,
			NULL), NULL);
	gst_app_src_set_stream_type(GST_APP_SRC_CAST(stream_source), GST_APP_STREAM_TYPE_STREAM);
	gst_app_src_set_latency(GST_APP_SRC_CAST(stream_source), 0, -1);
	g_object_set (G_OBJECT (stream_source),
		"format", GST_FORMAT_TIME,
		"is-live", true,
		"do-timestamp", true,
		NULL);

	/* Start playing */
    ret = gst_element_set_state (stream_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the stream pipeline to the playing state.\n");
        gst_object_unref (stream_pipeline);
        return;
    }

	stream_initialized = true;
}

void ROSGStreamer::init_recording() {
	ROS_INFO("Initializing recording with source video resolution %d x %d", original_width, original_height);

	std::string unique_path;
	determineFilePath(out_file, unique_path);

	/* set up the recording pipeline */
	// do it in here so we can more easily define the file name
	std::stringstream record_pipeline_ss;
	record_pipeline_ss << "appsrc name=source ! videoconvert ! "
				<< "videorate ! video/x-raw, framerate=(fraction)" << record_FPS << "/1 ! ";
	if (record_res_w)
	{
		record_pipeline_ss << "videoscale ! video/x-raw,width=" << record_res_w << " ! ";
	}
	record_pipeline_ss << "x264enc ! "
				<< "video/x-h264 ! "
				<< "matroskamux ! filesink location=" << unique_path;

	ROS_INFO("Recording pipeline: %s", record_pipeline_ss.str().c_str());

    GError *error = nullptr;
	record_pipeline = gst_parse_launch(record_pipeline_ss.str().c_str(), &error);
	if (error != nullptr) {
		ROS_ERROR("GST: %s", error->message);
		g_error_free(error);
		return;
	} else {
		ROS_INFO("gst_parse_launch successful for record pipeline");
	}

	record_source = gst_bin_get_by_name(GST_BIN(record_pipeline), "source");
	g_object_set (G_OBJECT (record_source), "caps",
		gst_caps_new_simple ("video/x-raw",
			"format", G_TYPE_STRING, "RGB",
			"width", G_TYPE_INT, original_width,
			"height", G_TYPE_INT, original_height,
			"framerate", GST_TYPE_FRACTION, 0, 1,
			NULL), NULL);
	gst_app_src_set_stream_type(GST_APP_SRC_CAST(record_source), GST_APP_STREAM_TYPE_STREAM);
	gst_app_src_set_latency(GST_APP_SRC_CAST(record_source), 0, -1);
	g_object_set (G_OBJECT (record_source),
		"format", GST_FORMAT_TIME,
		"is-live", true,
		"do-timestamp", true,
		NULL);

	// GstElement* record_sink = gst_bin_get_by_name(GST_BIN(record_pipeline), "filesink");
	// printf("d[%s]d\n", out_file.c_str());
	// // g_object_set (G_OBJECT (record_sink),
	// // 	"name", G_TYPE_STRING, "hello",
	// // 	NULL);
	// printf("d2\n");

	/* Start playing */
	ret = gst_element_set_state (record_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the record pipeline to the playing state.\n");
        gst_object_unref (record_pipeline);
        return;
    }

	recording = true;
}

void ROSGStreamer::stop_recording() {
	if (!recording)
		return;

	// indicate end of stream
	gst_app_src_end_of_stream(GST_APP_SRC(record_source));

	/* stop the pipeline */
	ret = gst_element_set_state (record_pipeline, GST_STATE_NULL);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to stop the record pipeline.\n");
        gst_object_unref (record_pipeline);
        return;
    } else {
		printf("Set recording to stopped\n");
	}
	recording = false;
}

void ROSGStreamer::recordCb(const std_msgs::Bool &msg) {
	if (msg.data) {
		if (recording) {
			ROS_WARN("Already recording");
			return;
		}
		init_recording();
	} else {
		if (recording)
			stop_recording();
	}
}

// determine a unique file name for the video file recorded to.
// if the file already exists, the file name will be appended by a number to make it unique
void ROSGStreamer::determineFilePath(const std::string &path, std::string &unique_path) {

	if (path[0] != '/') // not an absolute path
	{
		size_t size = 500;
		char cwd[size];
		auto ret = getcwd(cwd, size);
		ROS_INFO("Relative file path given. Will be interpreted relative to currend directory %s", cwd);
	}

	std::string name, ext;
	if (!separate_extension(path, name, ext)) // if file has no extension
		ext = "mkv";

	int cnt = 1;
	std::stringstream current_name;
	current_name << name << "." << ext;
	while(file_exists(current_name.str()))
	{
		current_name.str("");
		current_name << name << std::setw(3) << std::setfill('0') << cnt << "." << ext;
		cnt++;
	}
	unique_path = current_name.str();
}
