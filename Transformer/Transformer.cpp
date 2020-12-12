// Transformer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/pcl_macros.h>

int main()
{
    std::cout << "Hello World!\n";
    Eigen::AngleAxisd rotation_vector(M_PI / 5, Eigen::Vector3d(2, 2, 1).normalized());
    Eigen::Vector3d euler_angles = rotation_vector.matrix().eulerAngles(2, 1, 0);
    std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << std::endl;
    //Eigen::Vector3d euler_angles2 = rotation_vector.toRotationMatrix().eulerAngles(2, 1, 0);
    //std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles2.transpose() << std::endl;
    std::cout << "================================" << std::endl;
    //Eigen::Quaterniond q(0.0566210485547, -0.00721621889932, -0.561310569574, 0.825634560523);
    Eigen::Quaterniond q(0.0684957369773, 0.000577340300451, -0.565641570309, 0.821801444774);
    Eigen::Vector3d eulerAngle4 = q.matrix().eulerAngles(2, 1, 0);
    auto rpy = eulerAngle4.transpose();
    std::cout << "yaw(z) pitch(y) roll(x) = " << rpy << std::endl;
    std::cout << "yaw(z) pitch(y) roll(x) = " << rpy/M_PI *180 << std::endl;

    auto y_rect = std::atan(7.0 / 131.0);
    auto y_rect_angle = y_rect / M_PI * 180;
    std::cout << "y(rpy[1]) roll more : " << y_rect_angle << std::endl;
    auto yaw = rpy[0]+1.0/180*M_PI;
    auto pitch = rpy[1] ;
    auto roll = rpy[2] + y_rect;
    
    //1. 欧拉角 初始化
    Eigen::Vector3d eulerAngle(yaw, pitch, roll);

    //2. 欧拉角转旋转向量
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quaternion=yawAngle * pitchAngle * rollAngle;
    std::cout << "quaternion w: " << quaternion.w() << std::endl;
    std::cout << "quaternion x: " << quaternion.x() << std::endl;
    std::cout << "quaternion y: " << quaternion.y() << std::endl;
    std::cout << "quaternion z: " << quaternion.z() << std::endl;

    std::cout << "yaw(z) pitch(y) roll(x) = " << quaternion.matrix().eulerAngles(2,1,0).transpose() / M_PI * 180 << std::endl;
    std::cout << "================================" << std::endl;
    Eigen::AngleAxisd rotation_vector4_1;
    rotation_vector4_1 = q;
    std::cout<< "rotation_vector4 " << "angle is: " << rotation_vector4_1.angle() * (180 / M_PI)
        << " axis is: " << rotation_vector4_1.axis().transpose() << std::endl;

    Eigen::Matrix3d rotation_matrix4 = q.toRotationMatrix();
    std::cout << "rotation matrix4 =\n" << rotation_matrix4 << std::endl;

    std::cout << "================================" << std::endl;
    auto p_bottom = Eigen::Vector3f(1,1,1);
    auto p_top = Eigen::Vector3f(1,2,3);
    Eigen::Vector3f centerline_after = (p_top - p_bottom).normalized();
    Eigen::Vector3f centerline_before(0, 0, 1);
    Eigen::Vector3f centerline_axis = centerline_before.cross(centerline_after).normalized();
    float centerline_angle = std::acos(centerline_before.dot(centerline_after));
    Eigen::AngleAxisf centerline_angel_axis(centerline_angle, centerline_axis);
    Eigen::Matrix3f centerline_rotation = centerline_angel_axis.toRotationMatrix();
    Eigen::Quaternionf qq(centerline_angel_axis.toRotationMatrix());
    std::cout << qq.coeffs().transpose() << std::endl;
    qq.normalize();
    std::cout << qq.coeffs().transpose() << std::endl;



}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
