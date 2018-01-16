//#include "myslam/frame.h"
//#include "myslam/camera.h"
#include "myslam/buildmap.h"
namespace myslam
{
    octomap::Pointcloud buildmap::buildoctomap(Frame::Ptr frame)
    {
        frame_=frame;
        camera1_=frame->camera_;
        for(int v=0;v<frame_->color_.rows;v++)
            for(int u=0;u<frame_->color_.cols;u++)
            {
                unsigned int d=frame_->depth_.ptr<unsigned short>(v)[u];
                if(d==0) continue;
                if(d>=7000) continue;
                Vector3d point;
                point[2]=double(d)/camera1_->depth_scale_;
                point[0]=(u-camera1_->cx_)*point[2]/camera1_->fx_;
                point[1]=-(v-camera1_->cy_)*point[2]/camera1_->fy_;
                Vector3d pointWord=frame_->T_c_w_.rotation_matrix()*point;
                cloud.push_back(pointWord[0],pointWord[1],pointWord[2]);
                
            }
        
        return cloud;
    }

}