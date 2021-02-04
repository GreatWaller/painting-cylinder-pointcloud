// RotationAxisCalibration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

struct PositionAndOrientation
{
	double r_w;
	double r_x;
	double r_y;
	double r_z;

	double t_x;
	double t_y;
	double t_z;
};

void composeIsometry(PositionAndOrientation& model, Eigen::Isometry3d& result) {
	Eigen::Quaterniond q(model.r_w, model.r_x, model.r_y, model.r_z);
	Eigen::Vector3d t(model.t_x, model.t_y, model.t_z);
	result.prerotate(q);
	result.pretranslate(t);
}

int main()
{
	std::cout << "Hello World!\n";
	//圆一般方程 x^+y^+Dx+Ey+F=0
	// 1 取三点
	// 2 Ax=0
	// 3 求解

	// middle -left
	double r_w_l_0 = 0.6052752001382017;
	double r_x_l_0 = -0.35802992471940087;
	double r_y_l_0 = 0.3584243190605487;
	double r_z_l_0 = -0.6139939027458349;

	double t_x_l_0 = 0.2662479318438176;
	double t_y_l_0 = -0.19362955635484436;
	double t_z_l_0 = -0.02538322244306007;

	Eigen::Quaterniond q_l_0(r_w_l_0, r_x_l_0, r_y_l_0, r_z_l_0);
	Eigen::Vector3d t_l_0(t_x_l_0, t_y_l_0, t_z_l_0);
	Eigen::Isometry3d left_0(q_l_0);
	left_0.pretranslate(t_l_0);
	std::cout << "[left_0]"
		<< "\n"
		<< left_0.matrix() << std::endl;
	//std::cout << "[left_0_inverse]"
	//	<< "\n"
	//	<< left_0.inverse().matrix() << std::endl;

	//left 
	PositionAndOrientation leftModel_1;
	leftModel_1.r_w = 0.8358701637302538;
	leftModel_1.r_x = -0.48319278525953635;
	leftModel_1.r_y = 0.13267207233373313;
	leftModel_1.r_z = -0.2241515622996249;
	leftModel_1.t_x = 0.18515891566361;
	leftModel_1.t_y = -0.18739286654474152;
	leftModel_1.t_z = -0.03440378007871488;
	Eigen::Isometry3d left_1 = Eigen::Isometry3d::Identity();
	composeIsometry(leftModel_1, left_1);
	std::cout << "[left_1]"
		<< "\n"
		<< left_1.matrix() << std::endl;

	//middle-right
	double r_w_r_0 = 0.6093125968518028;
	double r_x_r_0 = -0.3588218664001429;
	double r_y_r_0 = -0.3605433880735749;
	double r_z_r_0 = 0.608270904143259;

	double t_x_r_0 = -0.26315823427648666;
	double t_y_r_0 = -0.3285345391280148;
	double t_z_r_0 = -0.02886998817732392;

	Eigen::Quaterniond q_r_0(r_w_r_0, r_x_r_0, r_y_r_0, r_z_r_0);
	Eigen::Vector3d t_r_0(t_x_r_0, t_y_r_0, t_z_r_0);
	Eigen::Isometry3d right_0(q_r_0);
	right_0.pretranslate(t_r_0);
	std::cout << "[right_0]"
		<< "\n"
		<< right_0.matrix() << std::endl;
	/*std::cout << "[right_0_inverse]"
		<< "\n"
		<< right_0.inverse().matrix() << std::endl;*/

	// right - left
	Eigen::Isometry3d leftbase2rightbase = right_0 * left_0.inverse();
	std::cout << "[left_right]"
		<< "\n"
		<< leftbase2rightbase.matrix() << std::endl;

	//right
	PositionAndOrientation rightModel_1;
	rightModel_1.r_w = 0.8522433182457819;
	rightModel_1.r_x = -0.45553103007714485;
	rightModel_1.r_y = -0.12848040155431872;
	rightModel_1.r_z = 0.22285778774526874;
	rightModel_1.t_x = -0.31270971865204533;
	rightModel_1.t_y = -0.2546463810059325;
	rightModel_1.t_z = -0.0671575865782397;
	Eigen::Isometry3d right_1 = Eigen::Isometry3d::Identity();
	composeIsometry(rightModel_1, right_1);

	Eigen::Isometry3d leftbase2right = leftbase2rightbase * right_1;
	std::cout << "[right_1]"
		<< "\n"
		<< leftbase2right.matrix() << std::endl;

	Eigen::Vector3d translation_left(left_1.translation()[0], left_1.translation()[1], left_1.translation()[2]);
	Eigen::Vector3d translation_middle(left_0.translation()[0], left_0.translation()[1], left_0.translation()[2]);
	Eigen::Vector3d translation_right(leftbase2right.translation()[0], leftbase2right.translation()[1], leftbase2right.translation()[2]);

	//A
	std::vector<Eigen::Vector3d> points;
	points.push_back(translation_left);
	points.push_back(translation_middle);
	points.push_back(translation_right);

	Eigen::MatrixXd A(5, 3);
	for (size_t i = 0; i < points.size(); i++)
	{
		A(i, 0) = points[i][0];
		A(i, 1) = points[i][1];
		A(i, 2) = 1;
	}
	//std::cout << "[" << A << "]" << std::endl;
	A.conservativeResize(points.size(), 3);
	std::cout << "[" << A << "]" << std::endl;

	// B
	Eigen::VectorXd B(points.size());
	for (size_t i = 0; i < points.size(); i++)
	{
		B[i] = -points[i][0] * points[i][0] - points[i][1] * points[i][1];
	}
	//B.conservativeResize(points.size());
	std::cout << "[" << B << "]" << std::endl;

	// solve
	//Eigen::Matrix<double, 3, 1> x;
	Eigen::VectorXd x(points.size());
	//x = A.colPivHouseholderQr().solve(B);
	x = A.inverse() * B;
	std::cout << "[" << x.transpose() << "]" << std::endl;

	// 圆心
	std::cout << "===================================" << std::endl;
	Eigen::Vector3d center(-x[0] / 2, -x[1] / 2, 0);
	std::cout << "[ center : " << center.transpose() << "]" << std::endl;
	double r = std::sqrt(x[0] * x[0] / 4 + x[1] * x[1] / 4 - x[2]);
	std::cout << "[ r = " << r << "]" << std::endl;
}
