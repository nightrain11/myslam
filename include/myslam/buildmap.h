#ifndef BUILDMAP_H
#define BUILDMAP_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include "myslam/frame.h"
namespace myslam
{
/* class Frame;
class Camera; */
class buildmap
{
private:
    Camera::Ptr camera1_;
    Frame::Ptr frame_;
    octomap::Pointcloud cloud;
public:
    buildmap() {}
    typedef std::shared_ptr<buildmap> Ptr;
    octomap::Pointcloud buildoctomap(Frame::Ptr frame);
};
}
#endif