// Transformer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/pcl_macros.h>
#include <atomic>

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

    std::cout << "================================" << std::endl;
    Eigen::Quaternionf q_take(0.6073137018021225, -0.5128772532430668, 0.3828096631747293, -0.4707268342566634);
    Eigen::Quaternionf q_set(0.646157637389, -0.464940288088, 0.353282820274, -0.49142861644);
    Eigen::Vector3f t_take(0.27385293090679175, -0.14985068977467274, -0.033165081109841126);
    Eigen::Vector3f t_set(0.260764601012, -0.155166573029, -0.0417871643352);
    auto r_take = q_take.toRotationMatrix();
    auto r_set = q_set.toRotationMatrix();
    Eigen::Vector3f t_s2t =  t_set- t_take;
    Eigen::Vector3f p_s_l(0.08521182467550453, -0.08137313680622592, 0.9382136827433396);
    Eigen::Vector3f p_t_l = (r_take.inverse())*(r_set * p_s_l + t_set-t_take);
    Eigen::Vector3f p_t_l_1 = q_take * q_set.inverse() * (p_s_l - t_set) + t_take;
    std::cout << "point p_s_l : " << p_t_l.transpose() << std::endl;
    std::cout << "point p_s_l_1 : " << p_t_l_1.transpose() << std::endl;
    std::cout << "=============Isometry===================" << std::endl;
    Eigen::Isometry3f t2w(q_take), s2w(q_set);
    t2w.pretranslate(t_take);
    s2w.pretranslate(t_set);
    //Eigen::Vector3f p_take = t2w * (s2w.inverse()) * p_s_l;
    Eigen::Vector3f p_take = t2w.inverse() * s2w * p_s_l;
    std::cout << "point p_take : " << p_take.transpose() << std::endl;
    std::cout << "=============Isometry===================" << std::endl;

    std::atomic_int a(0);
    a=5;
    std::cout << "a: "<< a << std::endl;

    std::cout << "=============quaternion test===================" << std::endl;
    Eigen::Matrix3d m3 =Eigen::Matrix3d::Identity();
    Eigen::Quaterniond q_3(m3);
    std::cout << "quaternion w: " << q_3.w() << std::endl;
    std::cout << "quaternion x: " << q_3.x() << std::endl;
    std::cout << "quaternion y: " << q_3.y() << std::endl;
    std::cout << "quaternion z: " << q_3.z() << std::endl;

    std::cout << "=============tf test===================" << std::endl;
	//specialPosition:
	//rotation:
	//w: 0.605275200138
	//x : -0.358029924719
	//y : 0.358424319061
	//z : -0.613993902746
	//translation :
	//    x : 0.266247931844
	//    y : -0.193629556355
	//	z : -0.0253832224431
    Eigen::Quaterniond q_0(0.605275200138, -0.358029924719, 0.358424319061, -0.613993902746);
    Eigen::Vector3d t_0(0.266247931844, -0.193629556355, -0.0253832224431);
    Eigen::Isometry3d camera2base(q_0);
    camera2base.pretranslate(t_0);

    std::cout << "camera to arm -> " << "\n" << camera2base.matrix() << std::endl;
    std::cout << "arm to camera " << "\n" << camera2base.inverse().matrix() << std::endl;
    //-Translation: [0.217, -0.260, -0.188]
    //    - Rotation : in Quaternion[0.000, -0.000, 0.707, 0.707]
    //    in RPY(radian)[-0.000, -0.000, 1.571]
    //    in RPY(degree)[-0.000, -0.000, 90.010]

    Eigen::Quaterniond pitch2base_q(0.000, -0.000, 0.707, 0.707);
    Eigen::Vector3d pitch2base_t(0.217, -0.260, -0.188);
    Eigen::Isometry3d pitch2base(pitch2base_q);
    pitch2base.pretranslate(pitch2base_t);

    std::cout << "camera to pitch 1" << "\n" << camera2base* pitch2base.matrix() << std::endl;
    std::cout << "camera to pitch 2" << "\n" << camera2base * pitch2base.inverse().matrix() << std::endl;
    std::cout << "camera to pitch 3-> true" << "\n" << camera2base.inverse() * pitch2base.matrix() << std::endl;
    std::cout << "camera to pitch 4" << "\n" << camera2base.inverse() * pitch2base.inverse().matrix() << std::endl;


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
