
#include "myslam/camera.h"
#include "myslam/frame.h"


namespace myslam
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
}

double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}

void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}
PointCloud::Ptr Frame::image2PointCloud()
{
    PointCloud::Ptr pointCloud( new PointCloud );
    PointCloud::Ptr current( new PointCloud );
    for(int m=0;m<depth_.rows;m++)
        for(int n=0;n<depth_.cols;n++)
        {
            ushort d=depth_.ptr<ushort>(m)[n];
            //d可能没有值，若如此，跳过此点
            if(d==0)
                continue;
            //d存在值，则向点云增加一个点
            PointT p;
            //d存在，则向点云增加一个点
            p.z=double(d)/camera_->depth_scale_;
            p.x=(n-camera_->cx_)*p.z/camera_->fx_;
            p.y=-(m-camera_->cy_)*p.z/camera_->fy_;
            //从RGB图像获取它的颜色
            p.b=color_.ptr<uchar>(m)[n*3];
            p.g=color_.ptr<uchar>(m)[n*3+1];
            p.r=color_.ptr<uchar>(m)[n*3+2];
            current->points.push_back(p);

        }
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter( *tmp );
        (*pointCloud) += *tmp;

        pointCloud->height=1;
        pointCloud->width=pointCloud->points.size();
        pointCloud->is_dense=false;
/*         current->height=1;
        current->width=current->points.size();
        current->is_dense=false; */
        return pointCloud;
}


}
