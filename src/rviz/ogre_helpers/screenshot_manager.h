
#ifndef OGRE_TOOLS_SCREENSHOTMANAGER_H
#define OGRE_TOOLS_SCREENSHOTMANAGER_H

#include <OgrePrerequisites.h>
#include <OgreRoot.h>
#include <OgreHardwarePixelBuffer.h>
// #include <OgreRenderWindow.h>

#include "rviz/render_panel.h"
namespace rviz
{
/* Class encapsulates Screenshot functionality and provides a method for making multi grid screenshots.    
   *  pRenderWindow:    Pointer to the render window.  This could be "mWindow" from the ExampleApplication,
   *              the window automatically created obtained when calling
   *              Ogre::Root::getSingletonPtr()->initialise(false) and retrieved by calling
   *              "Ogre::Root::getSingletonPtr()->getAutoCreatedWindow()", or the manually created
   *              window from calling "mRoot->createRenderWindow()".
   *  gridSize:      The magnification factor.  A 2 will create a 2x2 grid, doubling the size of the
                screenshot.  A 3 will create a 3x3 grid, tripling the size of the screenshot.
   *  fileExtension:    The extension of the screenshot file name, hence the type of graphics file to generate.
   *              To generate "MyScreenshot.png" this parameter would contain ".png".
   */
class ScreenshotManager
{
public:
  ScreenshotManager(Ogre::Root *root, RenderPanel *render_panel, int grid_size, Ogre::String folder, Ogre::String file_extension);
  ~ScreenshotManager();

  /* Creates a screenshot with the given camera.
    * @param camera Pointer to the camera "looking at" the scene of interest
    * @param file_name the filename of the screenshot file.
    * @param width of the screenshot file.
    * @param height of the screenshot file.
   */
  void makeScreenshot(Ogre::Camera *camera, Ogre::String file_name, Ogre::ColourValue bg_color, unsigned int width, unsigned int height);

protected:
  Ogre::Root *root_;
  Ogre::String folder_, file_extension_;
  unsigned int grid_size_, window_width_, window_height_;
  bool disable_overlays_;
  //temp texture with current screensize
  Ogre::TexturePtr temp_tex_;
  Ogre::RenderTexture *rt_;
  Ogre::HardwarePixelBufferSharedPtr buffer_;
  //PixelBox for a large Screenshot, if grid size is > 1
  Ogre::PixelBox final_picture_pb_;
  //Pointer to the color data of the pixel box
  Ogre::uint8 *data_;
  RenderPanel *render_panel_;
};
} // namespace rviz
#endif // OGRE_TOOLS_SCREENSHOTMANAGER_H