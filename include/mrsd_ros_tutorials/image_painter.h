/* 
 * Tutorial code courtesy of cv_bridge: 
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages#cv_bridge.2BAC8-Tutorials.2BAC8-UsingCvBridgeCppDiamondback.CA-ae53133f28c1a7955fa0e72fb63f62d02bdb25a8_1
 */
#ifndef __image_painter__
#define __image_painter__

#include <cv_bridge/cv_bridge.h>
/**
 * @brief namespace needed to disambiguate the in-source and abstracted classes
 */
namespace mrsd{

  /** 
   * @brief Class to paint onto images (demo of a fully abstracted class)
   * 
   * Very simple class to draw circles onto an image. Intended to demonstrate 
   * decoulping class abstraction from class invocation to help manage projects
   * better as they scal.
   */
  class ImagePainter
  {
    
  public:
    /** 
     *  @brief default constructor
     */
    ImagePainter(){};

    /** 
     *  @brief default destructor
     */
    ~ImagePainter(){};


    /** @brief paints colored circles onto input image
     *  The function draws circles along the diagional from upper left
     *  to lower right.  The color is hardcoded.
     *  @param cv_ptr Image to paint circles over
     */
    void do_really_cool_stuff(cv_bridge::CvImagePtr cv_ptr);
  };

} // namespace mrsd

#endif