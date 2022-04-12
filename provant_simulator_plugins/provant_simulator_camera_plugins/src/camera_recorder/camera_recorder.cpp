/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file camera_recorder.cpp
 * @brief This file contains the implementation of the CameraRecorder class.
 *
 * See the camera_recorder.h header for more detailed documentation.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_camera_plugins/camera_recorder/camera_recorder.h"

#include <ros/ros.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sstream>

using provant::camera_plugins::CameraRecorder;

CameraRecorder::CameraRecorder(gazebo::rendering::CameraPtr camera, const RecorderOptions& options,
                               const std::string logMsg, const std::string pluginId)
  : _camera(std::move(camera))
  , _options(options)
  , _logMsg(logMsg)
  , _imgMat(options.height.value_or(camera->ImageHeight()), options.width.value_or(camera->ImageWidth()), CV_8UC3)
  , _bgrMat(options.height.value_or(camera->ImageHeight()), options.width.value_or(camera->ImageWidth()), CV_8UC3)
  , _writer(options.path, options.fourcc, options.fps,
            cv::Size{ static_cast<int>(options.width.value_or(camera->ImageWidth())),
                      static_cast<int>(options.height.value_or(camera->ImageHeight())) },
            true)
  , _emptyFrameSlots(0)
  , _fullFrameSlots(0)
  , PLUGIN_ID(pluginId)
{
}

CameraRecorder::~CameraRecorder()
{
  _writer.release();
}

bool CameraRecorder::SetupCamera()
{
  if (_camera)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Configuring the " << _camera->Name() << " camera.");
    _camera->SetCaptureData(true);
    if (!_camera->CaptureData())
    {
      ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to set the " << _camera->Name()
                                                << "  to capture data.");
      return false;
    }

    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The " << _camera->Name() << " user camera has a resolution of "
                                             << _camera->ImageWidth() << "x" << _camera->ImageHeight() << ", "
                                             << _camera->ImageDepth() << " bits of color depth, uses "
                                             << _camera->ImageByteSize() << " bytes, and records in the "
                                             << _camera->ImageFormat() << " format.");

    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Connecting to the NewFrame signal.");
    _newFrameConn = _camera->ConnectNewImageFrame(
        std::bind(&provant::camera_plugins::CameraRecorder::OnNewFrame, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    return true;
  }

  return false;
}

void CameraRecorder::OnNewFrame(const unsigned char* image, unsigned int width, unsigned int height, unsigned int depth,
                                const std::string& format)
{
  ROS_DEBUG_STREAM_ONCE_NAMED(PLUGIN_ID, _logMsg << "A new frame was received!");

  try
  {
    if (_emptyFrameSlots.try_wait())
    {
      if (depth != 3)
      {
        ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An invalid image depth (" << depth << ") was received.");
        return;
      }

      ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Grabing a new frame.");
      CopyAndResize(image, width, height);
      if (!ConvertToBGR(format))
        return;
      SaveFrame();

      _fullFrameSlots.post();
    }
  }
  catch (const boost::interprocess::interprocess_exception& e)
  {
  }
}

const provant::camera_plugins::RecorderOptions& CameraRecorder::getOptions() const
{
  return _options;
}

bool CameraRecorder::GrabFrame()
{
  //  Notify that a new frame must be read
  try
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Posting the emptyFrameSlots semaphore.");
    _emptyFrameSlots.post();
    // Wait until a frame is read
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Waiting on the fullFrameSlots semaphore.");
    _fullFrameSlots.wait();
  }
  catch (const boost::interprocess::interprocess_exception& e)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An interprocess excpetion ocurred ath the GrabFrame method.");
  }

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Exiting the GrabFrame handler.");
  return true;
}

void CameraRecorder::CopyAndResize(const unsigned char* image, unsigned int width, unsigned int height)
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Copying the camera data.");
  if (width == _options.width && height == _options.height)
  {
    std::memcpy(_imgMat.data, image, _imgMat.total() * _imgMat.elemSize());
  }
  else
  {
    cv::Mat tempMat{ cv::Size{ static_cast<int>(width), static_cast<int>(height) }, CV_8UC3 };
    std::memcpy(tempMat.data, image, tempMat.total() * _imgMat.elemSize());

    const auto totalSrcSize = width * height;
    const auto totalDestSize = _imgMat.size().area();

    const auto interpolation =
        totalSrcSize > totalDestSize ? cv::InterpolationFlags::INTER_AREA : cv::InterpolationFlags::INTER_CUBIC;

    cv::resize(tempMat, _imgMat, _imgMat.size(), 0, 0, interpolation);
  }
}

bool CameraRecorder::ConvertToBGR(const std::string& format)
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Converting to the BGR color space.");

  if (format == "R8G8B8")
  {
    cv::cvtColor(_imgMat, _bgrMat, cv::COLOR_RGB2BGR);
  }
  else if (format == "B8G8R8")
  {
    // The image is already in the BGR color space
    _bgrMat = _imgMat.clone();
  }
  else if (format == "BAYER_BGGR8")
  {
    cv::cvtColor(_imgMat, _bgrMat, cv::COLOR_BayerBG2BGR);
  }
  else if (format == "BAYER_GBRG8")
  {
    cv::cvtColor(_imgMat, _bgrMat, cv::COLOR_BayerGB2BGR);
  }
  else if (format == "BAYER_RGGB8")
  {
    cv::cvtColor(_imgMat, _bgrMat, cv::COLOR_BayerRG2BGR);
  }
  else if (format == "BAYER_GRBG8")
  {
    cv::cvtColor(_imgMat, _bgrMat, cv::COLOR_BayerGR2BGR);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An image in a unrecognized format (" << format << ") was received.");
    return false;
  }

  return true;
}

void CameraRecorder::SaveFrame()
{
  if (_cont < _options.maxFrames)
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Saving the frame.");
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "The video writer is opened: " << _writer.isOpened());

    _writer.write(_bgrMat);

    _cont++;
  }
  else
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Releasing the video writer");
    _writer.release();
  }
}
