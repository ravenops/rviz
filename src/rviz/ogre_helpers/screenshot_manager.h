
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
      ScreenshotManager(Ogre::Root* root,RenderPanel* pRenderPanel, int gridSize, Ogre::String fileExtension, bool overlayFlag);
      ~ScreenshotManager();

   /* Creates a screenshot with the given camera.
    * @param camera Pointer to the camera "looking at" the scene of interest
    * @param fileName the filename of the screenshot file.
   */
      void makeScreenshot(Ogre::Camera* camera, Ogre::String fileName, Ogre::ColourValue bgColor, unsigned int width, unsigned int height);
     
   protected:
      Ogre::Root*     root ;
      Ogre::String     mFileExtension ;
      unsigned int   mGridSize, mWindowWidth, mWindowHeight;
      bool           mDisableOverlays;
      //temp texture with current screensize
      Ogre::TexturePtr mTempTex;
      Ogre::RenderTexture* mRT;
      Ogre::HardwarePixelBufferSharedPtr mBuffer;
      //PixelBox for a large Screenshot, if grid size is > 1
      Ogre::PixelBox  mFinalPicturePB;
      //Pointer to the color data of the pixel box
      Ogre::uint8* mData;
      RenderPanel* renderPanel;
    };
}
#endif  // OGRE_TOOLS_SCREENSHOTMANAGER_H