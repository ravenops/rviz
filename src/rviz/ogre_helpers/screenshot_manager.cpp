
#include "ogre_helpers/screenshot_manager.h"

#include <ros/console.h>
namespace rviz
{
ScreenshotManager::ScreenshotManager(Ogre::Root* pRoot, RenderPanel* pRenderPanel, int gridSize, Ogre::String fileExtension, bool overlayFlag)
{
    root = pRoot;
    //set file extension for the Screenshot files
    mFileExtension = fileExtension;
    // the gridsize
    mGridSize = gridSize;
    // flag for overlay rendering
    mDisableOverlays = overlayFlag;
    //get current window size
    renderPanel = pRenderPanel;    
}  

ScreenshotManager::~ScreenshotManager() 
{ 
    delete[] mData;
}

/* Creates a screenshot with the given camera.
* @param camera Pointer to the camera "looking at" the scene of interest
* @param fileName the filename of the screenshot file.
*/
void ScreenshotManager::makeScreenshot(Ogre::Camera *camera, Ogre::String fileName, Ogre::ColourValue bgColor, unsigned int width, unsigned int height)
{
    if(width != mWindowWidth || height != mWindowHeight){
        mWindowWidth = width;
        mWindowHeight = height;

        ROS_INFO("Resolution changes, updating screenshot manager %dx%d",width,height);
        mTempTex.setNull();
        mTempTex =  Ogre::TextureManager::getSingleton().createManual("ScreenShotTex",
                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
                                                           mWindowWidth, mWindowHeight, 0, Ogre::PF_B8G8R8, Ogre::TU_RENDERTARGET);

        //get The current Render Target of the temp Texture
        mRT = mTempTex->getBuffer()->getRenderTarget();

        //HardwarePixelBufferSharedPtr to the Buffer of the temp Texture
        mBuffer = mTempTex->getBuffer();

        //create PixelBox
        mData = new Ogre::uint8[(mWindowWidth * mGridSize) * (mWindowHeight * mGridSize) * 3];
        mFinalPicturePB = Ogre::PixelBox(mWindowWidth * mGridSize, mWindowHeight * mGridSize, 1, Ogre::PF_B8G8R8, mData);
    }

    //Remove all viewports, so the added Viewport(camera) ist the only
    mRT->removeAllViewports();
    mRT->addViewport(camera);

    //set the viewport settings
    Ogre::Viewport *vp = mRT->getViewport(0);
    vp->setClearEveryFrame(true);
    vp->setOverlaysEnabled(false);
    vp->setBackgroundColour(bgColor);

    if (mGridSize <= 1)
    {
        // Simple case where the contents of the screen are taken directly
        // Also used when an invalid value is passed within gridSize (zero or negative grid size)
        mRT->update(); //render

        //write the file on the Harddisk
        mRT->writeContentsToFile(fileName + "." + mFileExtension);
    }
    else
    {
        //define the original frustum extents variables
        Ogre::Real originalFrustumLeft, originalFrustumRight, originalFrustumTop, originalFrustumBottom;
        // set the original Frustum extents
        camera->getFrustumExtents(originalFrustumLeft, originalFrustumRight, originalFrustumTop, originalFrustumBottom);

        // compute the Stepsize for the drid
        Ogre::Real frustumGridStepHorizontal = (originalFrustumRight * 2) / mGridSize;
        Ogre::Real frustumGridStepVertical = (originalFrustumTop * 2) / mGridSize;

        // process each grid
        Ogre::Real frustumLeft, frustumRight, frustumTop, frustumBottom;
        for (unsigned int nbScreenshots = 0; nbScreenshots < mGridSize * mGridSize; nbScreenshots++)
        {
            int y = nbScreenshots / mGridSize;
            int x = nbScreenshots - y * mGridSize;

            // Shoggoth frustum extents setting
            // compute the new frustum extents
            frustumLeft = originalFrustumLeft + frustumGridStepHorizontal * x;
            frustumRight = frustumLeft + frustumGridStepHorizontal;
            frustumTop = originalFrustumTop - frustumGridStepVertical * y;
            frustumBottom = frustumTop - frustumGridStepVertical;

            // set the frustum extents value to the camera
            camera->setFrustumExtents(frustumLeft, frustumRight, frustumTop, frustumBottom);

            // ignore time duration between frames
            root->clearEventTimes();
            mRT->update(); //render

            //define the current
            Ogre::Box subBox = Ogre::Box(x * mWindowWidth, y * mWindowHeight, x * mWindowWidth + mWindowWidth, y * mWindowHeight + mWindowHeight);
            //copy the content from the temp buffer into the final picture PixelBox
            //Place the tempBuffer content at the right position
            mBuffer->blitToMemory(mFinalPicturePB.getSubVolume(subBox));
        }

        // set frustum extents to previous settings
        camera->resetFrustumExtents();

        Ogre::Image finalImage; //declare the final Image Object
        //insert the PixelBox data into the Image Object
        finalImage = finalImage.loadDynamicImage(static_cast<unsigned char *>(mFinalPicturePB.data), mFinalPicturePB.getWidth(), mFinalPicturePB.getHeight(), Ogre::PF_B8G8R8);
        // Save the Final image to a file
        finalImage.save(fileName + "." + mFileExtension);
    }

    // do we have to re-enable our overlays?
    if (mDisableOverlays){
        // myApplication::getSingletonPtr()->getDefaultViewport()->setOverlaysEnabled(true);
    }

    // reset time since last frame to pause the scene
    root->clearEventTimes();
}
} // namespace rviz