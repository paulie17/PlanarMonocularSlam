#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>

namespace pr {    

    typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
    typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
    typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > Matrix3fVector;
    typedef std::vector<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f> > Matrix2fVector;    

    // Matrix of floats, 2x3
    typedef Eigen::Matrix<float, 2, 3> Matrix2_3f; 
    // Matrix of floats, 4x6, ...
    typedef Eigen::Matrix<float, 4, 6> Matrix4_6f;
    typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
    typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

    typedef Eigen::Matrix<float, 6, 6> Matrix6f;
    typedef Eigen::Matrix<float, 6, 1> Vector6f;

    typedef std::vector<Vector6f, Eigen::aligned_allocator<Vector6f> > Vector6fVector;

    typedef Eigen::Matrix<float, 9, 6> Matrix9_6f;
    typedef Eigen::Matrix<float, 9, 9> Matrix9f;
    typedef Eigen::Matrix<float, 9, 1> Vector9f;

    typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;
    typedef Eigen::Matrix<float, 4, 3> Matrix4_3f;

    typedef std::vector<int> IntVector;

    typedef std::vector<float> FloatVector;

    typedef std::pair<int,int> IntPair;
    typedef std::vector<IntPair > IntPairVector;

    /*In C++, a template is a way to introduce a concept known as "generics". 
    With generics, you no longer need to concern your self with creating a function for each type, 
    the type will be deduced from the template function signature.*/

    template <class T> 
    bool isNan(const T& m){
        for (int i=0; i< m.rows(); i++) {
        for (int j=0; j< m.cols(); j++) {
        float v = m(i,j);
        if ( std::isnan( v ) )
        return true;
        }
        }
        return false;
    }


    inline Eigen::Isometry3f v2t(const Vector6f& t){
        Eigen::Isometry3f T;
        T.setIdentity();
        T.translation()=t.head<3>();
        float w=t.block<3,1>(3,0).squaredNorm();
        if (w<1) {
        w=sqrt(1-w);
        T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
        } else {
        T.linear().setIdentity();
        }
        return T;
    }

    inline Vector6f t2v(const Eigen::Isometry3f& t){
        Vector6f v;
        v.head<3>()=t.translation();
        Eigen::Quaternionf q(t.linear());
        v.block<3,1>(3,0)=q.matrix().block<3,1>(1,0);
        if (q.w()<0)
        v.block<3,1>(3,0) *= -1.0f;
        return v;
    }

    inline Eigen::Isometry2f v2t(const Eigen::Vector3f& t){
        Eigen::Isometry2f T;
        T.setIdentity();
        T.translation()=t.head<2>();
        float c = cos(t(2));
        float s = sin(t(2));
        T.linear() << c, -s, s, c;
        return T;
    }    

    inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
        Eigen::Vector3f v;
        v.head<2>()=t.translation();
        v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
        return v;
    }

    inline Eigen::Matrix3f Rx(float rot_x){
        float c=cos(rot_x);
        float s=sin(rot_x);
        Eigen::Matrix3f R;
        R << 1,  0, 0,
        0,  c,  -s,
        0,  s,  c;
        return R;
    }
    
    inline Eigen::Matrix3f Ry(float rot_y){
        float c=cos(rot_y);
        float s=sin(rot_y);
        Eigen::Matrix3f R;
        R << c,  0,  s,
        0 , 1,  0,
        -s,  0, c;
        return R;
    }

    inline Eigen::Matrix3f Rz(float rot_z){
        float c=cos(rot_z);
        float s=sin(rot_z);
        Eigen::Matrix3f R;
        R << c,  -s,  0,
        s,  c,  0,
        0,  0,  1;
        return R;
    }

    inline Eigen::Matrix3f dRz(float rot_z){
        float c=cos(rot_z);
        float s=sin(rot_z);
        Eigen::Matrix3f R;
        R << -s,  -c,  0,
        c,  -s,  0,
        0,  0,  1;
        return R;
    }

    
    inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
        Eigen::Isometry3f T;
        T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
        T.translation()=v.head<3>();
        return T;
    }


    inline Eigen::Matrix3f skew(const Eigen::Vector3f& v){
        Eigen::Matrix3f S;
        S << 0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0;
        return S;
    }

    inline Eigen::Isometry3f t2t3d(const Eigen::Isometry2f &iso){
        Eigen::Isometry3f _transform;
        _transform.setIdentity();
        _transform.linear().block<2, 2>(0, 0) = iso.linear();
        _transform.translation().block<2, 1>(0, 0) = iso.translation();
        return _transform;
    }

    inline Eigen::Vector4f to_homogeneous(const Eigen::Vector3f& v){
        return Eigen::Vector4f(v.x(), v.y(), v.z(), 1.0f);
    }    
}