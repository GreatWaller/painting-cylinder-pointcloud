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
	int div;
	int vane_count;
	float vane_width;
	int vane_div;
};

struct CircleModel {
	//单位法向量
	Eigen::Vector3f circleCenter;
	Eigen::Vector3f circelNorm;
	int div;
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
	int div;
};

int drawCylinder(pcl::PointCloud<pcl::PointXYZ>* cloud, CylinderModel& model, bool needCircle = 0) {
	int total = 0;

	Eigen::Vector3f cylinder_centerline_after = model.abovePoint - model.belowPoint;
	Eigen::Vector3f cylinder_centerline_origin(0., 0., 1);
	Eigen::Vector3f cylinder_traslation = model.belowPoint;
	//旋转角
	float l = std::sqrt(cylinder_centerline_after[0] * cylinder_centerline_after[0] + cylinder_centerline_after[1] * cylinder_centerline_after[1] + cylinder_centerline_after[2] * cylinder_centerline_after[2]);
	float cylinder_angle = std::acos(cylinder_centerline_after[2] / l);

	//法向量:既作为转轴，同时可判断夹角是否为逆时针
	Eigen::Vector3f cylinder_axis = cylinder_centerline_origin.cross(cylinder_centerline_after);
	if (cylinder_axis[2] < 0.0)
		cylinder_angle = -cylinder_angle;
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

			pcl::PointXYZ point;
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
					pcl::PointXYZ point;
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

int drawCircle(pcl::PointCloud<pcl::PointXYZ>* cloud, CircleModel& model) {
	int count = 0;
	Eigen::Vector3f circle_norm_before(0, 0, 1);
	float angle = std::acos(circle_norm_before.dot(model.circelNorm));
	Eigen::Vector3f axis = circle_norm_before.cross(model.circelNorm);
	if (axis[2] < 0)
	{
		angle = -angle;
	}
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

		pcl::PointXYZ point;
		point.x = tt[0];
		point.y = tt[1];
		point.z = tt[2];
		++count;
		cloud->points.push_back(point);
	}
	cloud->width += count;
	return count;
}

int drawHook(pcl::PointCloud<pcl::PointXYZ>* cloud, HookModel& model) {
	int count = 0;

	/// 1 直线
	/// 2 1/2 圆
	/// 3 1/4 圆
	std::vector<Eigen::Vector3f> hooks;
	for (size_t i = 1; i <= model.div; i++)
	{
		float x = model.line_length * i / model.div;
		pcl::PointXYZ point;
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
		pcl::PointXYZ point;
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
		pcl::PointXYZ point;
		point.x = x;
		point.y = y;
		point.z = z;
		Eigen::Vector3f t(x, y, z);
		hooks.push_back(t);
	}

	Eigen::Vector3f hook_after_ahead_n = (model.p_b_left - model.p_a_left).cross(model.p_c_right - model.p_a_left).normalized();
	Eigen::Vector3f hook_before_ahead_n(0, 0, 1);
	Eigen::Vector3f hook_axis = hook_before_ahead_n.cross(hook_after_ahead_n);
	float hook_angle = std::acos(hook_before_ahead_n.dot(hook_after_ahead_n));
	if (hook_axis[2] < 0)
	{
		hook_angle = -hook_angle;
	}
	Eigen::AngleAxisf hook_angel_axis(hook_angle, hook_axis);
	auto hook_rotation = hook_angel_axis.toRotationMatrix();
	auto hook_translation = (model.p_a_left + model.p_c_right) / 2;

	for (auto& p : hooks) {
		for (size_t i = 0; i < model.div; i++)
		{
			float h_s = i < model.div / 2 ? std::asin((i + 1) / ((float)model.div / 2)) * (model.d_between_hooks / M_PI) : (M_PI - std::asin((model.div - i - 1) / (float)model.div * 2)) * (model.d_between_hooks / M_PI);

			Eigen::Vector3f t_t_hoot(0, -model.d_between_hooks / 2 + h_s, 0);
			Eigen::Vector3f l_t = hook_rotation * (p + t_t_hoot) + hook_translation;

			pcl::PointXYZ point;
			point.x = l_t[0];
			point.y = l_t[1];
			point.z = l_t[2];

			++count;
			cloud->points.push_back(point);
			cloud->width += count;
		}
	}
	return count;
}

#endif // !_MODEL_DRAWING_H_