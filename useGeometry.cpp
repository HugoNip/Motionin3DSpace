#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <cmath>

int main (int argc, char** argv)
{
    // 3D Rotation
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
    std::cout.precision(3);
    std::cout << "rotation matrix = \n" << rotation_vector.matrix() << std::endl;

    // use AngleAxis
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    std::cout << "(1, 0, 0) after rotation (by angle axis) = " << v_rotated.transpose() << std::endl;

    // use rotation matrix
    rotation_matrix = rotation_vector.toRotationMatrix();
    v_rotated = rotation_matrix * v;
    std::cout << "(1, 0, 0) after rotation (by matrix) = " << v_rotated.transpose() << std::endl;

    // Euler angle
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl;

    // Euler transformation
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(1, 3, 4));
    std::cout << "Transofrm matrix = \n" << T.matrix() << std::endl;

    // Transform matrix
    Eigen::Vector3d v_transformed = T * v;
    std::cout << "v transformed = " << v_transformed.transpose() << std::endl;

    // Quaterniond
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl;

    q = Eigen::Quaterniond(rotation_matrix);
    std::cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << std::endl;

    // Rotation
    v_rotated = q * v;
    std::cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << std::endl;

    // General Method
    std::cout << "should be equal to " << (q * Eigen::Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << std::endl;

    return 0;
}
