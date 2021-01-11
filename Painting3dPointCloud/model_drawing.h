#ifndef _MODEL_DRAWING_H_
#define _MODEL_DRAWING_H_


#include <Eigen/Dense>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。

struct CylinderModel
{
	Eigen::Vector3f abovePoint;
	Eigen::Vector3f belowPoint;
	float r;
	float h;
	int div=128;
	int vane_count;
	float vane_width;
	int vane_div;
};

struct CircleModel {
	//单位法向量
	Eigen::Vector3f circleCenter;
	Eigen::Vector3f circelNorm;
	int div=128;
	float r;
};

struct HookModel
{
	Eigen::Vector3f p_a_left;
	Eigen::Vector3f p_b_left;
	Eigen::Vector3f p_c_right;
	float line_length = 1.f;
	float d;
	float d_between_hooks;
	int div=128;
	int h_div = 2;
};

struct SphereModel
{
	Eigen::Vector3f center;
	float radius;
	uint8_t r=255;
	uint8_t g;
	uint8_t b;
	int div=128;
};

struct HexagonModel
{
	Eigen::Vector3f center=Eigen::Vector3f(0,0,1);
	Eigen::Vector3f centerline = Eigen::Vector3f(0, 0, 1);
	float r;
	float h;
	int div = 16;
	int h_div = 8;
};

int drawCylinder(pcl::PointCloud<pcl::PointXYZRGB>* cloud, CylinderModel& model, bool needCircle = 0) {
	int total = 0;

	Eigen::Vector3f cylinder_centerline_after = model.abovePoint - model.belowPoint;
	Eigen::Vector3f cylinder_centerline_origin(0., 0., 1);
	Eigen::Vector3f cylinder_traslation = model.belowPoint;
	//旋转角
	float l = std::sqrt(cylinder_centerline_after[0] * cylinder_centerline_after[0] + cylinder_centerline_after[1] * cylinder_centerline_after[1] + cylinder_centerline_after[2] * cylinder_centerline_after[2]);
	float cylinder_angle = std::acos(cylinder_centerline_after[2] / l);

	//法向量:既作为转轴，同时可判断夹角是否为逆时针(应该不需要判断)
	Eigen::Vector3f cylinder_axis = cylinder_centerline_origin.cross(cylinder_centerline_after);
	//if (cylinder_axis[2] < 0.0)
	//	cylinder_angle = -cylinder_angle;
	Eigen::AngleAxisf cylinder_angle_axis(cylinder_angle, cylinder_axis.normalized());
	Eigen::Matrix3f cylinder_rotation = cylinder_angle_axis.toRotationMatrix();

	for (size_t j = 0; j <= model.div; j++)
	{
		for (size_t i = 0; i <= model.div; i++)
		{
			float alpha = 2. * M_PI * i / model.div;
			float x = model.r * sin(alpha);
			float y = model.r * cos(alpha);
			float z = model.h * j / model.div;

			Eigen::Vector3f t(x, y, z);
			Eigen::Vector3f tt = cylinder_rotation * t + cylinder_traslation;

			pcl::PointXYZRGB point;

			point.g = 255;

			point.x = tt[0];
			point.y = tt[1];
			point.z = tt[2];
			++total;
			cloud->points.push_back(point);
		}
	}
	if (needCircle)
	{
		//circle
		for (size_t i = 1; i <= model.vane_count; i++)
		{
			float z = model.h * i / (model.vane_count + 1);
			for (size_t j = 0; j < model.vane_div; j++)
			{
				for (size_t k = 0; k < model.div; k++)
				{
					float alpha = 2. * M_PI * k / model.div;
					float x = (model.r + model.vane_width * j / model.vane_div) * sin(alpha);
					float y = (model.r + model.vane_width * j / model.vane_div) * cos(alpha);

					Eigen::Vector3f t(x, y, z);
					Eigen::Vector3f tt = cylinder_rotation * t + cylinder_traslation;
					pcl::PointXYZRGB point;

					point.g = 255;
				
					point.x = tt[0];
					point.y = tt[1];
					point.z = tt[2];
					++total;
					cloud->points.push_back(point);
				}
			}
		}
	}

	return total;
}

int drawCircle(pcl::PointCloud<pcl::PointXYZRGB>* cloud, CircleModel& model) {
	int count = 0;
	Eigen::Vector3f circle_norm_before(0, 0, 1);
	float angle = std::acos(circle_norm_before.dot(model.circelNorm.normalized()));
	Eigen::Vector3f axis = circle_norm_before.cross(model.circelNorm).normalized();

	Eigen::AngleAxisf angleAxis(angle, axis);
	Eigen::Matrix3f rotation = angleAxis.toRotationMatrix();
	Eigen::Vector3f translation = model.circleCenter;

	for (size_t i = 1; i <= model.div; i++)
	{
		float alpha = 2. * M_PI * i / model.div;
		float x = model.r * sin(alpha);
		float y = model.r * cos(alpha);
		float z = 0;

		Eigen::Vector3f t(x, y, z);
		Eigen::Vector3f tt = rotation * t + translation;

		pcl::PointXYZRGB point;
		point.x = tt[0];
		point.y = tt[1];
		point.z = tt[2];
		++count;
		cloud->points.push_back(point);
	}
	return count;
}

int drawHook(pcl::PointCloud<pcl::PointXYZRGB>* cloud, HookModel& model) {
	int count = 0;

	/// 1 直线
	/// 2 1/2 圆
	/// 3 1/4 圆
	std::vector<Eigen::Vector3f> hooks;
	for (size_t i = 1; i <= model.div; i++)
	{
		float x = model.line_length * i / model.div;
		pcl::PointXYZRGB point;
		point.x = x;
		point.y = 0;
		point.z = 0;
		Eigen::Vector3f t(x, 0, 0);
		hooks.push_back(t);
	}

	for (size_t i = 1; i <= model.div; i++)
	{
		float x = model.d * i / model.div + model.line_length;
		float z = -std::sqrt((model.d / 2) * (model.d / 2) - (model.d * i / model.div - model.d / 2) * (model.d * i / model.div - model.d / 2));
		float y = 0;
		pcl::PointXYZRGB point;
		point.x = x;
		point.y = y;
		point.z = z;
		Eigen::Vector3f t(x, y, z);
		hooks.push_back(t);
	}

	for (size_t i = 1; i <= model.div; i++)
	{
		float alpha = M_PI * i / 2 / model.div;
		float x = model.d * sin(alpha) + model.line_length;
		float z = model.d * cos(alpha);
		float y = 0;
		pcl::PointXYZRGB point;
		point.x = x;
		point.y = y;
		point.z = z;
		Eigen::Vector3f t(x, y, z);
		hooks.push_back(t);
	}
	Eigen::Vector3f hook_left_after = (model.p_b_left - model.p_a_left).normalized();
	Eigen::Vector3f hook_left_before(1, 0, 0);
	Eigen::Vector3f hook_left_axis = hook_left_before.cross(hook_left_after).normalized();
	float hook_left_angle = std::acos(hook_left_before.dot(hook_left_after));

	Eigen::AngleAxisf hook_left_angel_axis(hook_left_angle, hook_left_axis);
	auto hook_left_rotation = hook_left_angel_axis.toRotationMatrix();


	Eigen::Vector3f hook_origin_ahead_n(0, 0, 1);
	Eigen::Vector3f hook_before_ahead_n = hook_left_rotation * hook_origin_ahead_n.normalized();
	Eigen::Vector3f hook_after_ahead_n = (model.p_b_left - model.p_a_left).cross(model.p_c_right - model.p_a_left).normalized();
	Eigen::Vector3f hook_axis = hook_before_ahead_n.cross(hook_after_ahead_n).normalized();
	float hook_angle = std::acos(hook_before_ahead_n.dot(hook_after_ahead_n));
	//float a1 = hook_axis[2];
	//float a2 = hook_left_after[2];
	//auto t1 = *(int*)&a1 ^ *(int*)&a2;
	if ((*(int*)&(hook_axis[2]) ^ *(int*)&(hook_left_after[2])) < 0)
	{
		hook_angle = -hook_angle;
	}
	Eigen::AngleAxisf hook_angel_axis(hook_angle, hook_left_after);
	auto hook_rotation = hook_angel_axis.toRotationMatrix();



	Eigen::Vector3f hook_translation = (model.p_a_left + model.p_c_right) / 2;

	for (auto& p : hooks) {
		for (size_t i = 0; i <= model.h_div; i++)
		{
			float h_s = i < model.h_div / 2 ? std::asin((i ) / ((float)model.h_div / 2)) * (model.d_between_hooks / M_PI) : (M_PI - std::asin((model.h_div - i - 1) / (float)model.h_div * 2)) * (model.d_between_hooks / M_PI);

			Eigen::Vector3f t_t_hoot(0, -model.d_between_hooks / 2 + h_s, 0);
			Eigen::Vector3f l_t = hook_rotation*hook_left_rotation * (p + t_t_hoot) + hook_translation;

			pcl::PointXYZRGB point;
			point.r = 255;
			point.g = 255;
			point.b = 255;
			point.x = l_t[0];
			point.y = l_t[1];
			point.z = l_t[2];

			++count;
			cloud->points.push_back(point);
		}
	}
	return count;
}

int drawSphere(pcl::PointCloud<pcl::PointXYZRGB>* cloud, SphereModel& model) {
	int count = 0;
	for (size_t j = 1; j <= model.div; j++)
	{
		float z = 2. * model.radius * j / model.div - model.radius;

		for (size_t i = 1; i <= model.div; i++)
		{
			float alpha = 2. * M_PI * i / model.div;
			float rPrime = sqrt(model.radius * model.radius - z * z);
			float x = rPrime * sin(alpha);
			float y = rPrime * cos(alpha);

			pcl::PointXYZRGB point;
			point.r = model.r;
			point.g = model.g;
			point.b = model.b;
			point.x = x+model.center[0];
			point.y = y+model.center[1];
			point.z = z+model.center[2];
			++count;
			cloud->points.push_back(point);
		}
	}
	return count;
}

int drawHexagon(pcl::PointCloud<pcl::PointXYZRGB>* cloud, HexagonModel& model) {
	int count = 0;
	Eigen::Vector3f nut_centerline = model.centerline.normalized();
	Eigen::Vector3f nut_base(0, 0, 1);
	Eigen::Vector3f nut_axis = nut_base.cross(nut_centerline).normalized();
	float nut_angle = std::acos(nut_base.dot(nut_centerline));

	Eigen::AngleAxisf nut_angel_axis(nut_angle, nut_axis);
	Eigen::Matrix3f nut_rotation = nut_angel_axis.toRotationMatrix();
	Eigen::Vector3f nut_translation = model.center - Eigen::Vector3f(0, 0, model.h / 2);

	for (size_t h = 0; h <= model.h_div; h++)
	{
		float z = model.h * h / model.h_div;
		for (size_t i = 0; i <= model.div; i++)
		{
			float x1 = (0.5 + 0.5 * i / model.div) * model.r;
			float y1 = -std::sqrt(3.0) * (x1 - model.r);
			float x2 = -x1;
			float y2 = y1;
			float x3 = -x1;
			float y3 = -y1;
			float x4 = x1;
			float y4 = -y1;
			float x5 = (-0.5 + (1.0) * i / model.div) * model.r;
			float y5 = std::sqrt(3.0) / 2 * model.r;
			float x6 = x5;
			float y6 = -y5;

			Eigen::Vector3f sixPoints[6];
			sixPoints[0] = Eigen::Vector3f(x1, y1, z);
			sixPoints[1] = Eigen::Vector3f(x2, y2, z);
			sixPoints[2] = Eigen::Vector3f(x3, y3, z);
			sixPoints[3] = Eigen::Vector3f(x4, y4, z);
			sixPoints[4] = Eigen::Vector3f(x5, y5, z);
			sixPoints[5] = Eigen::Vector3f(x6, y6, z);

			for (size_t i = 0; i < 6; i++)
			{
				Eigen::Vector3f tt = nut_rotation * sixPoints[i] + nut_translation;
				pcl::PointXYZRGB point;
				point.x = tt[0];
				point.y = tt[1];
				point.z = tt[2];
				point.g = 255;
				cloud->points.push_back(point);
				++count;
			}
		}
	}
	
	return count;
}

#endif // !_MODEL_DRAWING_H_