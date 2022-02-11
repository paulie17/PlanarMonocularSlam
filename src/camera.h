#pragma once
#include "defs.h"

namespace pr {
  /**
     simple pinhole camera class.
     Has
     - the position (camera with respect to world)
     - the camera matrix
     - the size of the image plane in pixels
     Supports simple projection operations
  */
  class Camera{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //~ ctor, initialized a camera according to the arguments
    Camera(int rows=100,
           int cols=100,
           const Eigen::Matrix3f& camera_matrix=Eigen::Matrix3f::Identity(),
           const Eigen::Isometry3f& camera_to_world_pose=Eigen::Isometry3f::Identity());

    //! projects a single point on the image plane
    //! @returns false if the point is behind the camera or outside the plane
    inline  bool projectPoint(Eigen::Vector2f& image_point,
                              const Eigen::Vector3f& world_point){
      Eigen::Vector3f camera_point=_camera_to_world_pose.inverse()*world_point;
      if (camera_point.z()<=0)
        return false;
      Eigen::Vector3f projected_point=_camera_matrix*camera_point;
      image_point=projected_point.head<2>()*(1./projected_point.z());
      if(image_point.x()<0 || image_point.x()>_cols-1){
        return false;}
      if(image_point.y()<0 || image_point.y()>_rows-1){
        return false;}
      return true;
    }

    //! projects a bunch of world points on the image
    //! @param image_points: the points on the image
    //! @param world_points: the input world points
    //! @param keep_indices: if true, image_points has the same size of world points
    //! Invalid points are marked with (-1, -1). If false only the points in the set are returned
    //! @returns the number of points that fall inside the image
    int projectPoints(Vector2fVector& image_points,
                      const Vector3fVector& world_points,
                      bool keep_indices=false);

    const Eigen::Vector3f bearing_from_img_point(const Eigen::Vector2f& img_point); 
    const Vector3fVector bearings_from_img_points(const Vector2fVector& img_points);                      
  
    inline const Eigen::Isometry3f& cameraToWorldPose() const {return _camera_to_world_pose;}
    inline void setCameraToWorldPose(const Eigen::Isometry3f& pose)  {_camera_to_world_pose=pose;}
    inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix;}
    inline void setCameraMatrix(const Eigen::Matrix3f& matrix)  {_camera_matrix = matrix;}
    inline void setImageSize(const int height, const int width)  {_rows = width; _cols = height;}
    inline void printImageSize() {std::cout << "width: " << _cols << "\n";std::cout << "height: " << _rows << "\n"; }

  protected:
    int _rows; // image_size
    int _cols; // 
    Eigen::Matrix3f _camera_matrix;
    Eigen::Isometry3f _camera_to_world_pose;
  };
}
