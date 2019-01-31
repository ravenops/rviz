/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QApplication>
#include <QDir>
#include <QTimer>

#include <boost/program_options.hpp>

#include <OgreMaterialManager.h>
#include <OgreGpuProgramManager.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <std_srvs/Empty.h>
#include <boost/filesystem.hpp>


#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
// Apparently OSX #defines 'check' to be an empty string somewhere.
// That was fun to figure out.
#undef check
#endif

#include <ros/console.h>
#include <ros/ros.h>

#include "rviz/selection/selection_manager.h"
#include "rviz/env_config.h"
#include "rviz/ogre_helpers/ogre_logging.h"
#include "rviz/visualization_frame.h"
#include "rviz/visualization_manager.h"
#include "rviz/wait_for_master_dialog.h"
#include "rviz/ogre_helpers/render_system.h"
#include "rviz/dump_images_config.h"

#include "rviz/visualizer_app.h"

#define CATCH_EXCEPTIONS 0

namespace po = boost::program_options;

namespace rviz
{

bool reloadShaders(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  ROS_INFO("Reloading materials.");
  {
    Ogre::ResourceManager::ResourceMapIterator it = Ogre::MaterialManager::getSingleton().getResourceIterator();
    while (it.hasMoreElements())
    {
      Ogre::ResourcePtr resource = it.getNext();
      resource->reload();
    }
  }
  ROS_INFO("Reloading high-level gpu shaders.");
  {
    Ogre::ResourceManager::ResourceMapIterator it = Ogre::HighLevelGpuProgramManager::getSingleton().getResourceIterator();
    while (it.hasMoreElements())
    {
      Ogre::ResourcePtr resource = it.getNext();
      resource->reload();
    }
  }
  ROS_INFO("Reloading gpu shaders.");
  {
    Ogre::ResourceManager::ResourceMapIterator it = Ogre::GpuProgramManager::getSingleton().getResourceIterator();
    while (it.hasMoreElements())
    {
      Ogre::ResourcePtr resource = it.getNext();
      resource->reload();
    }
  }
  return true;
}

VisualizerApp::VisualizerApp()
    : app_(0), continue_timer_(0), frame_(0)
{
}

std::map<std::string,std::string> VisualizerApp::env_param_names =
    {
     {"RVN_RVIZ_DUMP_IMAGES","dump-images"},
     {"RVN_RVIZ_CAPTURED_OUT", "captured-out"},
     {"RVN_RVIZ_KEYED_OUT", "keyed-out"},
     {"RVN_RVIZ_THUMB_OUT", "thumb-out"},
     {"RVN_RVIZ_POSTER_OUT", "poster-out"},
     {"RVN_RVIZ_THUMB_WIDTH", "thumb-width"},
     {"RVN_RVIZ_DUMP_FPS_NUM", "dump-fps-num"},
     {"RVN_RVIZ_DUMP_FPS_DEN", "dump-fps-den"},
     {"RVN_RVIZ_DUMP_TIMEOUT", "dump-timeout"},
     {"RVN_RVIZ_MAX_WIDTH", "max-width"},
    };

void VisualizerApp::setApp(QApplication *app)
{
  app_ = app;
}

bool VisualizerApp::init(int argc, char **argv)
{
  ROS_INFO("rviz version %s", get_version().c_str());
  ROS_INFO("compiled against Qt version " QT_VERSION_STR);
  ROS_INFO("compiled against OGRE version %d.%d.%d%s (%s)",
           OGRE_VERSION_MAJOR, OGRE_VERSION_MINOR, OGRE_VERSION_PATCH,
           OGRE_VERSION_SUFFIX, OGRE_VERSION_NAME);

#ifdef Q_OS_MAC
  ProcessSerialNumber PSN;
  GetCurrentProcess(&PSN);
  TransformProcessType(&PSN, kProcessTransformToForegroundApplication);
  SetFrontProcess(&PSN);
#endif

#if CATCH_EXCEPTIONS
  try
  {
#endif
    ros::init(argc, argv, "rviz", ros::init_options::AnonymousName);

    startContinueChecker();

    po::options_description options;
       options.add_options()
      ("help,h", "Produce this help message")
      ("splash-screen,s", po::value<std::string>(), "A custom splash-screen image to display")
      ("help-file", po::value<std::string>(), "A custom html file to show as the help screen")
      ("display-config,d", po::value<std::string>(), "A display config file (.rviz) to load")
      ("fixed-frame,f", po::value<std::string>(), "Set the fixed frame")
      ("ogre-log,l", "Enable the Ogre.log file (output in cwd) and console output.")
      ("in-mc-wrapper", "Signal that this is running inside a master-chooser wrapper")
      ("opengl", po::value<int>(), "Force OpenGL version (use '--opengl 210' for OpenGL 2.1 compatibility mode)")
      ("disable-anti-aliasing", "Prevent rviz from trying to use anti-aliasing when rendering.")
      ("no-stereo", "Disable the use of stereo rendering.")
      ("verbose,v", "Enable debug visualizations")
      ("log-level-debug", "Sets the ROS logger level to debug.")
      ("dump-images", "On every screen render dump a jpg of contents.")
      ("captured-out",po::value<std::string>()->default_value("captured.mp4"), "Output path for captured mp4 file")
      ("keyed-out",po::value<std::string>()->default_value("captured_keyed.mp4"), "Output path for captured keyed mp4 file")
      ("thumb-out",po::value<std::string>()->default_value("captured_thumbnail.jpg"), "Output path for thumbnail jpg file")
      ("thumb-width",po::value<int>()->default_value(128), "width for thumbnail")
      ("poster-out",po::value<std::string>()->default_value("captured_poster.jpg"), "Output path for poster (fullsize thumbnail) jpg file")
      ("max-width", po::value<int>()->default_value(1920), "maximum width of outputted encoded mp4 video streams")
      ("dump-fps-num",po::value<int>()->default_value(30), "Numerator of frames per second for dumped x264 stream.  Must be an integer.")
      ("dump-fps-den",po::value<int>()->default_value(1), "Denominator number of frames per second for dumped x264 stream.  Must be an integer.")
      ("dump-timeout",po::value<float>()->default_value(30), "Retry initial connection to DBus for X seconds failing.");

    po::variables_map vm;
    std::string display_config, fixed_frame, splash_path, help_path;
    bool enable_ogre_log = false;
    bool in_mc_wrapper = false;
    bool verbose = false;

    int force_gl_version = 0;
    bool disable_anti_aliasing = false;
    bool disable_stereo = false;
    DumpImagesConfig* dump_images_config = NULL;

    try
    {
      // parse environment variables
      po:store(po::parse_environment
               (options,
                [](const std::string& i_env_var){
                    std::map<std::string,std::string>::iterator it = VisualizerApp::env_param_names.find(i_env_var);
                    if(it != VisualizerApp::env_param_names.end()){
                        //element found;
                        ROS_INFO("[RVN] Using environment variable %s",i_env_var.c_str());
                        return it->second;
                    }
                    return std::string("");
                }),vm);
      // parse command line args. These get precedence over environment variables
      po::store(po::parse_command_line(argc, argv, options), vm);
      po::notify(vm);

      if (vm.count("help"))
      {
        std::cout << "rviz command line options:\n"
                  << options;
        return false;
      }

      if (vm.count("in-mc-wrapper"))
      {
        in_mc_wrapper = true;
      }

      if (vm.count("display-config"))
      {
        display_config = vm["display-config"].as<std::string>();
        if (display_config.size() >= 4 && display_config.substr(display_config.size() - 4, 4) == ".vcg")
        {
          std::cerr << "ERROR: the config file '" << display_config << "' is a .vcg file, which is the old rviz config format." << std::endl;
          std::cerr << "       New config files have a .rviz extension and use YAML formatting.  The format changed" << std::endl;
          std::cerr << "       between Fuerte and Groovy.  There is not (yet) an automated conversion program." << std::endl;
          return false;
        }
      }

      if (vm.count("splash-screen"))
      {
        splash_path = vm["splash-screen"].as<std::string>();
      }

      if (vm.count("help-file"))
      {
        help_path = vm["help-file"].as<std::string>();
      }

      if (vm.count("fixed-frame"))
      {
        fixed_frame = vm["fixed-frame"].as<std::string>();
      }

      if (vm.count("ogre-log"))
      {
        enable_ogre_log = true;
      }

      if (vm.count("no-stereo"))
      {
        disable_stereo = true;
      }

      if (vm.count("opengl"))
      {
        //std::cout << vm["opengl"].as<std::string>() << std::endl;
        force_gl_version = vm["opengl"].as<int>();
      }

      if (vm.count("disable-anti-aliasing"))
      {
        disable_anti_aliasing = true;
      }

      if (vm.count("verbose"))
      {
        verbose = true;
      }

      if (vm.count("log-level-debug"))
      {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
          ros::console::notifyLoggerLevelsChanged();
        }
      }

      if (vm.count("dump-images"))
      {
        dump_images_config = new DumpImagesConfig;
        dump_images_config->enabled = true;
        dump_images_config->captured_path = vm["captured-out"].as<std::string>();
        dump_images_config->keyed_path = vm["keyed-out"].as<std::string>();
        dump_images_config->thumb_path = vm["thumb-out"].as<std::string>();
        dump_images_config->thumbWidth = vm["thumb-width"].as<int>();
        dump_images_config->poster_path = vm["poster-out"].as<std::string>();
        dump_images_config->maxWidth = vm["max-width"].as<int>();
        dump_images_config->fpsNum = vm["dump-fps-num"].as<int>();
        dump_images_config->fpsDen = vm["dump-fps-den"].as<int>();
        dump_images_config->timeout = vm["dump-timeout"].as<float>();
        dump_images_config->frameWidth = ((float) dump_images_config->fpsDen) / ((float) dump_images_config->fpsNum);
        dump_images_config->bagDuration = -1.0;
        dump_images_config->nextTime = 0;
        dump_images_config->lastEventTime = -1.0;
        dump_images_config->preloadDuration = -1.0;
      }
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Error parsing command line: %s", e.what());
      return false;
    }

    if (!ros::master::check())
    {
      WaitForMasterDialog *dialog = new WaitForMasterDialog;
      if (dialog->exec() != QDialog::Accepted)
      {
        return false;
      }
    }

    nh_.reset(new ros::NodeHandle);

    if (enable_ogre_log)
    {
      OgreLogging::useRosLog();
    }

    if (force_gl_version)
    {
      RenderSystem::forceGlVersion(force_gl_version);
    }

    if (disable_anti_aliasing)
    {
      RenderSystem::disableAntiAliasing();
    }

    if (disable_stereo)
    {
      RenderSystem::forceNoStereo();
    }

    frame_ = new VisualizationFrame();
    frame_->setApp(this->app_);
    if (help_path != "")
    {
      frame_->setHelpPath(QString::fromStdString(help_path));
    }
    frame_->setShowChooseNewMaster(in_mc_wrapper);
    if (vm.count("splash-screen"))
    {
      frame_->setSplashPath(QString::fromStdString(splash_path));
    }

    frame_->initialize(QString::fromStdString(display_config), dump_images_config);
    if (!fixed_frame.empty())
    {
      frame_->getManager()->setFixedFrame(QString::fromStdString(fixed_frame));
    }

    frame_->getManager()->getSelectionManager()->setDebugMode(verbose);

    frame_->show();

    ros::NodeHandle private_nh("~");
    reload_shaders_service_ = private_nh.advertiseService("reload_shaders", reloadShaders);

    ROS_INFO("[RVN] rviz version v%s", get_version().c_str());
#if CATCH_EXCEPTIONS
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Caught exception while loading: %s", e.what());
    return false;
  }
#endif
  return true;
}

VisualizerApp::~VisualizerApp()
{
  delete continue_timer_;
  delete frame_;
}

void VisualizerApp::startContinueChecker()
{
  continue_timer_ = new QTimer(this);
  connect(continue_timer_, SIGNAL(timeout()), this, SLOT(checkContinue()));
  continue_timer_->start(100);
}

void VisualizerApp::checkContinue()
{
  if (!ros::ok())
  {
    if (frame_)
    {
      // Make sure the window doesn't ask if we want to save first.
      frame_->setWindowModified(false);
    }
    QApplication::closeAllWindows();
  }
}

} // namespace rviz
