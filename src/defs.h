#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <iostream>

namespace pms {    
    
    typedef std::vector<Eigen::Triplet<double>, Eigen::aligned_allocator<Eigen::Triplet<double>>  > tripletList;

    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dVector;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dVector;

    // Matrix of doubles, 2x3
    typedef Eigen::Matrix<double, 2, 3> Matrix2_3d; 
    // Matrix of doubles, 4x6, ...
    typedef Eigen::Matrix<double, 6, 3> Matrix6_3d;

    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    typedef std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > Vector6dVector;

    typedef Eigen::Matrix<double, 6, 3> Matrix6_3d;

    typedef std::vector<int> IntVector;

    typedef std::vector<double> DoubleVector;

    typedef std::pair<int,int> IntPair;
    typedef std::vector<IntPair > IntPairVector;

    /*In C++, a template is a way to introduce a concept known as "generics". 
    With generics, you no longer need to concern your self with creating a function for each type, 
    the type will be deduced from the template function signature.*/

    template <class T> 
    bool isNan(const T& m){
        for (int i=0; i< m.rows(); i++) {
        for (int j=0; j< m.cols(); j++) {
        double v = m(i,j);
        if ( std::isnan( v ) )
        return true;
        }
        }
        return false;
    }

    inline Eigen::Isometry2d v2t(const Eigen::Vector3d& t){
        Eigen::Isometry2d T;
        T.setIdentity();
        T.translation()=t.head<2>();
        double c = cos(t(2));
        double s = sin(t(2));
        T.linear() << c, -s, s, c;
        return T;
    }    

    inline Eigen::Vector3d t2v(const Eigen::Isometry2d& t){
        Eigen::Vector3d v;
        v.head<2>()=t.translation();
        v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
        return v;
    }

    inline Eigen::Matrix3d dRz(double rot_z){
        double c=cos(rot_z);
        double s=sin(rot_z);
        Eigen::Matrix3d R;
        R << -s,  -c,  0,
        c,  -s,  0,
        0,  0,  1;
        return R;
    }

    inline Eigen::Matrix2d dR_2d(double rot){
        double c=cos(rot);
        double s=sin(rot);
        Eigen::Matrix2d R;
        R << -s,  -c,
        c,  -s;
        return R;
    }

    inline Eigen::Isometry3d t2t3d(const Eigen::Isometry2d &iso){
        Eigen::Isometry3d _transform;
        _transform.setIdentity();
        _transform.linear().block<2, 2>(0, 0) = iso.linear();
        _transform.translation().block<2, 1>(0, 0) = iso.translation();
        return _transform;
    }

    inline Eigen::VectorXd flatten(const Eigen::MatrixXd& m){
        Eigen::VectorXd flattened(m.size());
        for (int c = 0; c < m.cols(); c++) {
            flattened.segment(c * m.rows(), m.rows()) = m.col(c);
        }
        return flattened;
    }

}