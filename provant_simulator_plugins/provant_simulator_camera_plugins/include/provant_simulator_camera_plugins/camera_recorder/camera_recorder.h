/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file camera_recorder.h
 * @brief This file contains the declaration of the RecorderOptions struct and of the CameraRecorder class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CAMERA_RECORDER_H
#define PROVANT_CAMERA_RECORDER_H

#include <gazebo/common/Events.hh>
#include <gazebo/rendering/rendering.hh>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include <limits>
#include <memory>
#include <optional>

namespace provant
{
namespace camera_plugins
{
/**
 * @brief Options used to configure a synchronized video recorder object.
 */
struct RecorderOptions
{
  //! Absolute path to the resulting video file. Must contain the mp4 extension.
  std::string path;
  //! Horizontal resolution of the video.
  std::optional<unsigned int> width;
  //! Vertical resolution of the video.
  std::optional<unsigned int> height;
  //! Desired video frame rate, must be an integer dividend of the simulation frequency.
  double fps;
  //! Maximum number of simulation steps to record. The video recording will be stoped after this number.
  std::optional<unsigned int> maxFrames;
  //! Codec used to encode the video. The default uses the H.264 codec.
  //! Note: I tried using H.265, but the results are inconsistent, and there is no way to check if a given codec is
  //! usable by opencv.
  int fourcc = cv::VideoWriter::fourcc('A', 'V', 'C', '1');
};

/**
 * @brief
 *
 * @todo Add class documentation.
 * @todo Refactor to receive a spdlog::logger pointer instead of the _logMessage and PLUGIN_ID values.
 *
 */
class CameraRecorder
{
public:
  /**
   * @brief Construct a new Camera Recorder object.
   *
   * @param camera Pointer to a Gazebo camera from which the frames will be captured.
   * @param options Set of options used to configure the recorder, such as the path of the resulting video file,
   * desired resolution and fps.
   * @param logMsg Message used to uniquely identify the log messages emitted by this object.
   * @param pluginId Name of the ROS child logger used by this object.
   */
  CameraRecorder(gazebo::rendering::CameraPtr camera, const RecorderOptions& options, const std::string logMsg,
                 const std::string pluginId);
  /**
   * @brief Destroy the Camera Recorder object and ensures that the video writer object was correctly closed, thus
   * ensuring that the video was correctly saved to file.
   *
   * Note: Due to the strange behavior of Gazebo when a request is received, it is not possible to ensure that the
   * destructor will be called, thus ensuring the video was saved to the file, for this reason, it is recommended
   * that the users configure a value for the maximum number of frames that must be captured by the recorder, thus
   * allowing the video to be saved without a call to this destructor.
   */
  virtual ~CameraRecorder();

  /**
   * @brief Configure the camera to capture frames, and connect the appropriate signals for the correct behavior
   * of the plugin.
   *
   * @return true If the camera configuration was successfull.
   * @return false Otherwise.
   */
  bool SetupCamera();

  /**
   * @brief Get the Options object.
   *
   * Return a copy of the options object used to configure this recorder.
   *
   * @return const RecorderOptions&
   */
  const RecorderOptions& getOptions() const;

  /**
   * @brief Capture a new frame from the camera managed by this object.
   *
   * @return true If the frame was successfully captured.
   * @return false Otherwise.
   */
  bool GrabFrame();

protected:
  /**
   * @brief Handler of the NewFrame event of the camera managed by this object. Called every time a new frame is
   * available from the camera.
   *
   * This method will try to decrement the _emptyFrameSlots sempahore, and if it successfull, will proceed to copy
   * the image to an OpenCV Mat object, resize it to the user configured resolution, convert to the adequate color
   * space and encode it and save it to the resulting video file.
   *
   * @param image Pointer to the binary image data.
   * @param width Horizontal resolution of the captured image.
   * @param height Vertical resolution of the captured image.
   * @param depth Number of color channels available in the captured image.
   * @param format Color format (space) of the captured image.
   */
  virtual void OnNewFrame(const unsigned char* image, unsigned int width, unsigned int height, unsigned int depth,
                          const std::string& format);

  /**
   * @brief Copy the captured frame image to an OpenCV Mat object, and if necessary resizes the image to the configured
   * resolution.
   *
   * @param image Pointer to the binary image data.
   * @param width Horizontal resolution of the captured image.
   * @param height Vertical resolution of the captured image.
   */
  virtual void CopyAndResize(const unsigned char* image, unsigned int width, unsigned int height);
  /**
   * @brief Convert the captured frame to the BGR color space.
   *
   * This method converts the captured frame available in the _imgMat property to the BGR color space expected by
   * the OpenCV tools, and puts the converted image in the _bgrMat object.
   *
   * This method can accept images in the RGB, BGR, and Bayer color formats.
   *
   * @param format The image format of the camera.
   */
  virtual bool ConvertToBGR(const std::string& format);
  /**
   * @brief Encode and write a new saved frame to the video file.
   *
   * The video file is read from the _bgrMat property, and must be in the BGR color space.
   */
  virtual void SaveFrame();

private:
  //! Pointer to the camera from which the frames are captured.
  gazebo::rendering::CameraPtr _camera;
  //! Connection to the NewFrame event, trigerred by the camera when a new frame is available.
  gazebo::event::ConnectionPtr _newFrameConn;

  //! Options used to configure the recorder. Contains elements such as the destination filepath, resolution and fps.
  const RecorderOptions _options;
  //! OpenCV Mat object that contains the original frame captured from the camera.
  cv::Mat _imgMat;
  /**
   * @brief OpenCV Mat object that contains the capture frame converted to the BGR colorspace.
   *
   * As the gazebo cameras provide the image data in the RGB colorspace, and OpenCV expects the images to in the BGR
   * colorspace, a color space conversion is necessary in order to ensure that the resulting video file has the
   * correct colors.
   */
  cv::Mat _bgrMat;
  //! Video writer used to encode and save the video frames to a file.
  cv::VideoWriter _writer;

  //! Number of frames captured by this recorder.
  unsigned int _cont = 0;

  /**
   * @brief Semaphore used to indicate the number of available frames slots.
   *
   * When this semaphore has a non zero value it indicates that a new frame must be saved from the camera.
   */
  boost::interprocess::interprocess_semaphore _emptyFrameSlots;
  /**
   * @brief Semaphore used to indicate the number of caputred frames.
   *
   * When this semaphore has a non zero value it indicates that a frame was saved from the camera, and the world
   * update process can continue.
   */
  boost::interprocess::interprocess_semaphore _fullFrameSlots;

  //! Message used to uniquely identify the log messages emitted by this object in the log output.
  const std::string _logMsg;
  //! Name of the ROS child logger used by this object.
  const std::string PLUGIN_ID;
};
}  // namespace camera_plugins
}  // namespace provant

#endif  // PROVANT_CAMERA_RECORDER_H
