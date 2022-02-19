#include "camera.h"

namespace pms {
  Camera::Camera(int rows,
                 int cols,
                 const Eigen::Matrix3f& camera_matrix,
                 const Eigen::Isometry3f& camera_to_world_pose):
    _rows(rows),
    _cols(cols),
    _camera_matrix(camera_matrix),
    _camera_to_world_pose(camera_to_world_pose){}


  int Camera::projectPoints(Vector2fVector& image_points,
                            const Vector3fVector& world_points,
                            bool keep_indices){
    image_points.resize(world_points.size());
    int num_image_points=0;
    const Eigen::Vector2f point_outside(-1,-1);
    int num_points_inside=0;
    for(size_t i=0; i<world_points.size(); i++){
      const Eigen::Vector3f world_point=world_points[i];
      Eigen::Vector2f& image_point=image_points[num_image_points];
      bool is_inside=projectPoint(image_point,world_point);
      if (is_inside)
        num_points_inside++;
      else
        image_point=point_outside;
      if (keep_indices||is_inside){
        num_image_points++;
      } 
    }
    image_points.resize(num_image_points);
    return num_points_inside;
  }

  const Eigen::Vector3f Camera::bearing_from_img_point(const Eigen::Vector2f& img_point){
    Eigen::Vector3f bearing;
    Eigen::Vector3f img_point_extended;

    img_point_extended.head(2) = img_point;
    img_point_extended(2) = 1.;
    
    bearing = _camera_matrix.inverse()*img_point_extended;        
    bearing = bearing / bearing.norm();
    bearing = _camera_to_world_pose.rotation()*bearing;
    
    return bearing;
  }

  const Vector3fVector Camera::bearings_from_img_points(const Vector2fVector& img_points){
    Vector3fVector bearings;
    Eigen::Vector3f bearing;
    int n_pts;

    n_pts = img_points.size();
    for (int i = 0; i < n_pts; i++){      
      bearing = bearing_from_img_point(img_points[i]);
      bearings.push_back(bearing);
    }
    return bearings;
  }

}
