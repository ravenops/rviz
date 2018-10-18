
#include "ogre_helpers/screenshot_manager.h"

#include <ros/console.h>
namespace rviz
{
ScreenshotManager::ScreenshotManager(Ogre::Root *root, RenderPanel *render_panel, int grid_size, Ogre::String folder, Ogre::String file_extension)
{
    root_ = root;
    folder_ = folder;
    file_extension_ = file_extension;
    grid_size_ = grid_size;
    render_panel_ = render_panel;
}

ScreenshotManager::~ScreenshotManager()
{
    delete[] data_;
}

/* Creates a screenshot with the given camera.
* @param camera Pointer to the camera "looking at" the scene of interest
* @param fileName the filename of the screenshot file.
*/
void ScreenshotManager::makeScreenshot(Ogre::Camera *camera, Ogre::String file_name, Ogre::ColourValue bg_color, unsigned int width, unsigned int height)
{
    if (window_width_ != width || window_height_ != height)
    {
        window_width_ = width;
        window_height_ = height;

        ROS_INFO("Resolution changes, updating screenshot manager %dx%d", width, height);
        temp_tex_.setNull();
        temp_tex_ = Ogre::TextureManager::getSingleton().createManual("ScreenShotTex",
                                                                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
                                                                      window_width_, window_height_, 0, Ogre::PF_B8G8R8, Ogre::TU_RENDERTARGET);

        //get The current Render Target of the temp Texture
        rt_ = temp_tex_->getBuffer()->getRenderTarget();

        //HardwarePixelBufferSharedPtr to the Buffer of the temp Texture
        buffer_ = temp_tex_->getBuffer();

        //create PixelBox
        data_ = new Ogre::uint8[(window_width_ * grid_size_) * (window_height_ * grid_size_) * 3];
        final_picture_pb_ = Ogre::PixelBox(window_width_ * grid_size_, window_height_ * grid_size_, 1, Ogre::PF_B8G8R8, data_);
    }

    //Remove all viewports, so the added Viewport(camera) ist the only
    rt_->removeAllViewports();
    rt_->addViewport(camera);

    //set the viewport settings
    Ogre::Viewport *vp = rt_->getViewport(0);
    vp->setClearEveryFrame(true);
    vp->setBackgroundColour(bg_color);

    Ogre::String full_path = folder_ + "/" + file_name + "." + file_extension_;
    if (grid_size_ <= 1)
    {
        // Simple case where the contents of the screen are taken directly
        // Also used when an invalid value is passed within gridSize (zero or negative grid size)
        rt_->update(); //render

        //write the file on the Harddisk
        rt_->writeContentsToFile(full_path);
    }
    else
    {
        //define the original frustum extents variables
        Ogre::Real original_frustum_left, original_frustum_right, original_frustum_top, original_frustum_bottom;
        // set the original Frustum extents
        camera->getFrustumExtents(original_frustum_left, original_frustum_right, original_frustum_top, original_frustum_bottom);

        // compute the Stepsize for the drid
        Ogre::Real frustum_grid_step_horizontal = (original_frustum_right * 2) / grid_size_;
        Ogre::Real frustum_grid_step_vertical = (original_frustum_top * 2) / grid_size_;

        // process each grid
        Ogre::Real frustum_left, frustum_right, frustum_top, frustum_bottom;
        for (unsigned int screenshots_count = 0; screenshots_count < (grid_size_ * grid_size_); screenshots_count++)
        {
            int y = screenshots_count / grid_size_;
            int x = screenshots_count - y * grid_size_;

            // Shoggoth frustum extents setting
            // compute the new frustum extents
            frustum_left = original_frustum_left + frustum_grid_step_horizontal * x;
            frustum_right = frustum_left + frustum_grid_step_horizontal;
            frustum_top = original_frustum_top - frustum_grid_step_vertical * y;
            frustum_bottom = frustum_top - frustum_grid_step_vertical;

            // set the frustum extents value to the camera
            camera->setFrustumExtents(frustum_left, frustum_right, frustum_top, frustum_bottom);

            // ignore time duration between frames
            root_->clearEventTimes();
            rt_->update(); //render

            //define the current
            Ogre::Box sub_box = Ogre::Box(x * window_width_, y * window_height_, x * window_width_ + window_width_, y * window_height_ + window_height_);
            //copy the content from the temp buffer into the final picture PixelBox
            //Place the tempBuffer content at the right position
            buffer_->blitToMemory(final_picture_pb_.getSubVolume(sub_box));
        }

        // set frustum extents to previous settings
        camera->resetFrustumExtents();

        Ogre::Image final_image; //declare the final Image Object
        //insert the PixelBox data into the Image Object
        final_image = final_image.loadDynamicImage(
            static_cast<unsigned char *>(final_picture_pb_.data),
            final_picture_pb_.getWidth(),
            final_picture_pb_.getHeight(),
            Ogre::PF_B8G8R8);
        // Save the Final image to a file
        final_image.save(full_path);
    }

    // reset time since last frame to pause the scene
    root_->clearEventTimes();
}
} // namespace rviz