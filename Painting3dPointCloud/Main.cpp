#include <pcl/visualization/cloud_viewer.h>
#include <iostream>//标准C++库中的输入输出类相关头文件。
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include<fstream>  
#include <string>  
#include <vector> 
#include <Eigen/Dense>

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
	float line_length;
	float d;
	float d_between_hooks;
	int div;
};

int drawCylinder(pcl::PointCloud<pcl::PointXYZ>* cloud, CylinderModel& model,bool needCircle=0) {
	int total = 0;

	Eigen::Vector3f cylinder_centerline_after = model.abovePoint-model.belowPoint;
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
					float alpha = 2. * M_PI * k /model.div;
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
	if (axis[2]<0)
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
	int count=0;

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

int main() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	
	//pcl::io::savePCDFileASCII("txt2pcd_bunny1.pcd", *cloud);
	//std::cerr << "Saved " << cloud->points.size() << " data points to txt2pcd.pcd." << std::endl;
	long num_points = 0;
	//ring
	CircleModel ring;
	ring.circelNorm = Eigen::Vector3f(1, 1, -1);
	ring.circleCenter= Eigen::Vector3f(1, 1, -1);
	ring.r = 0.5;
	ring.div = 128;
	int ringPoints = drawCircle(cloud.get(), ring);

	//hook
	HookModel hookModel;
	hookModel.line_length = 0.5f;
	hookModel.d = 0.2f;
	hookModel.div = 64;
	hookModel.d_between_hooks = 0.2f;
	hookModel.p_a_left = Eigen::Vector3f(-1, -1, 1);
	hookModel.p_b_left = Eigen::Vector3f(-1, -1, 0);
	hookModel.p_c_right = Eigen::Vector3f(0, 0, 1);
	drawHook(cloud.get(), hookModel);
	//cylinder
	float r = .025;
	int div = 200;
	float h = .205f;
	

	//通过 angleAxis 求 rotation 
	// 在原点做圆柱 取（0，0，1）为原轴
	// 目标轴为 两点 之差
	//1 计算法两直线的法向量，为转轴，
	// 2 计算旋转角度 为转角；
	/// 3 计算 rotation 3*3
	/// 4 计算 translation 3*1;
	Eigen::Vector3f v_a(1., 1., 1.);
	Eigen::Vector3f v_b(2., 2., 3.);
	Eigen::Vector3f v_n = v_b - v_a;

	Eigen::Vector3f v_o(0., 0., 1);
	Eigen::Vector3f t_t = v_a ;
	//旋转角：点乘
	float l = std::sqrt(v_n[0] * v_n[0] + v_n[1] * v_n[1] + v_n[2] * v_n[2]);
	float ax = std::acos(v_n[2] / l);
	
	//法向量:既作为转轴，同时可作为夹角是否为逆时针
	Eigen::Vector3f v_c= v_o.cross(v_n);
	if (v_c[2] < 0.0)
		ax = -ax;
	Eigen::AngleAxisf aa(ax, v_c.normalized());
	Eigen::Matrix3f t_r = aa.toRotationMatrix();
	Eigen::Vector3f vt(0, 0, 2 * sqrt(3));
	Eigen::Vector3f ttttt = t_r * vt + t_t;
	for (size_t j = 0; j <= div; j++)
	{
		for (size_t i = 0; i <= div; i++)
		{
			float alpha = 2. * M_PI * i / div;
			float x = r * sin(alpha);
			float y = r * cos(alpha);
			float z = h * j / div;

			Eigen::Vector3f t(x, y, z);
			Eigen::Vector3f tt = t_r * t + t_t;

			pcl::PointXYZ point;
			point.x = tt[0];
			point.y = tt[1];
			point.z = tt[2];
			++num_points;
			cloud->points.push_back(point);
		}
	}

	//circle
	int num_vane = 5;
	float width_vane = .0235f;
	int d_v = 20;
	for (size_t i = 1; i <= num_vane; i++)
	{
		float z = h * i / (num_vane+1);
		for (size_t j = 0; j < d_v; j++)
		{
			for (size_t k = 0; k < div; k++)
			{
				float alpha = 2. * M_PI * k / div;
				float x = (r + width_vane * j / d_v) * sin(alpha);
				float y = (r + width_vane * j / d_v) * cos(alpha);

				Eigen::Vector3f t(x, y, z);
				Eigen::Vector3f tt = t_r * t + t_t;
				pcl::PointXYZ point;
				point.x = tt[0];
				point.y = tt[1];
				point.z = tt[2];
				++num_points;
				cloud->points.push_back(point);
			}
			
		}

	}

	//hook
	std::vector<Eigen::Vector3f> hooks;

	/// 1 直线
	/// 2 1/2 圆
	/// 3 1/4 圆
	float line_length = .0474f;
	for (size_t i = 1; i <= div; i++)
	{
		float x = line_length * i / div;
		pcl::PointXYZ point;
		point.x = x;
		point.y = 0;
		point.z = 0;
		Eigen::Vector3f t(x, 0, 0);
		hooks.push_back(t);
		/*++num_points;
		cloud->points.push_back(point);*/
	}

	float d = .013f;
	for (size_t i = 1; i <= div; i++)
	{
		float x = d * i / div + line_length;
		float z = -std::sqrt((d / 2) * (d / 2) - (d * i / div-d/2) * (d * i / div - d / 2));
		float y = 0;
		pcl::PointXYZ point;
		point.x = x;
		point.y = y;
		point.z = z;
		Eigen::Vector3f t(x, y, z);
		hooks.push_back(t);

		/*++num_points;
		cloud->points.push_back(point);*/
	}

	for (size_t i = 1; i <= div; i++)
	{
		float alpha = M_PI * i /2 / div;
		float x = d * sin(alpha) + line_length;
		float z = d * cos(alpha);
		float y = 0;
		pcl::PointXYZ point;
		point.x = x;
		point.y = y;
		point.z = z;
		Eigen::Vector3f t(x, y, z);
		hooks.push_back(t);

		/*++num_points;
		cloud->points.push_back(point);*/
	}

	/// 1 圆点偏移
	/// 2 圆点旋转
	/// 3 跟随圆柱旋转
	/// 4 钩子平面法向量移动后的与实测法向量夹角
	float d_between_hooks = 0.05f;
	Eigen::Vector3f left_hook(0.112, -d_between_hooks/2, -0.053);
	Eigen::Vector3f right_hook(0.112, d_between_hooks/2, -0.053);
	Eigen::AngleAxisf aa_hook(M_PI * 2 / 5, Eigen::Vector3f(0, 1, 0));
	Eigen::Matrix3f t_r_hoot = aa_hook.toRotationMatrix();

	Eigen::Vector3f v_n_before(0, -1, 0);
	Eigen::Vector3f v_n_after = t_r * v_n_before + t_t;
	//实际测量的平面三点
	float x0 = 1.0;
	float y0 = 1.0;
	float z0 = 1.0;
	float x1 = 2.0;
	float y1 = 2.0;
	float z1 = 2.0;
	float x2 = 1.0;
	float y2 = 2.0;
	float z2 = 3.0;
	
	Eigen::Vector3f pa(x0, y0, z0);
	Eigen::Vector3f pb(x1, y1, z1);
	Eigen::Vector3f pc(x2, y2, z2);

	//1 钩子与绝缘子一体
	Eigen::Vector3f m_n = (pb - pa).cross(pc - pa);
	float cosineValue= m_n.dot(v_n_after-v_a) / (m_n.norm()*(v_n_after-v_a).norm());
	float beta = std::acos(cosineValue);
	Eigen::Vector3f o_n = (v_n_after - v_a).cross(m_n);
	if (o_n[2]<0)
	{
		beta = -beta;
	}
	Eigen::AngleAxisf aa_final(beta, v_n.normalized());
	Eigen::Matrix3f t_r_hoot_final = aa_final.toRotationMatrix();

	int dd = 10;
	for (auto& p : hooks) {
		for (size_t i = 0; i < dd; i++)
		{
			//float aaa = std::asin(0.1);
			//float h_s = std::asin((i + 1) / (float)dd ) * (d_between_hooks / M_PI);

			float h_s =i<dd/2? std::asin((i + 1) / ((float)dd/2)) * (d_between_hooks / M_PI): (M_PI - std::asin((dd-i - 1) / (float)dd*2)) * (d_between_hooks / M_PI);

			Eigen::Vector3f t_t_hoot(0.112, -d_between_hooks / 2 + h_s, -0.053);
			Eigen::Vector3f l_t = t_r_hoot_final*(t_r * (t_r_hoot * p + t_t_hoot)) + t_t;

			pcl::PointXYZ point;
			point.x = l_t[0];
			point.y = l_t[1];
			point.z = l_t[2];

			++num_points;
			cloud->points.push_back(point);
		}

		/*Eigen::Vector3f l_t = t_r * (t_r_hoot * p + left_hook) + t_t;

		pcl::PointXYZ point;
		point.x = l_t[0];
		point.y = l_t[1];
		point.z = l_t[2];

		++num_points;
		cloud->points.push_back(point);

		Eigen::Vector3f r_t = t_r*(t_r_hoot*p + right_hook)+t_t;

		pcl::PointXYZ r_point;
		r_point.x = r_t[0];
		r_point.y = r_t[1];
		r_point.z = r_t[2];

		++num_points;
		cloud->points.push_back(r_point);*/
	}
	//2 钩子单独画 (0,0,1)为钩子平面初始法向量
	Eigen::Vector3f hook_after_ahead_n = (pb - pa).cross(pc - pa).normalized();
	Eigen::Vector3f hook_before_ahead_n(0, 0, 1);
	Eigen::Vector3f hook_axis = hook_before_ahead_n.cross(hook_after_ahead_n);
	float hook_angle =std::acos(hook_before_ahead_n.dot(hook_after_ahead_n));
	if (hook_axis[2]<0)
	{
		hook_angle = -hook_angle;
	}
	Eigen::AngleAxisf hook_angel_axis(hook_angle, hook_axis);
	auto hook_rotation = hook_angel_axis.toRotationMatrix();
	auto hook_translation = (pa + pc) / 2;

	//int dd = 10;
	for (auto& p : hooks) {
		for (size_t i = 0; i < dd; i++)
		{
			float h_s = i < dd / 2 ? std::asin((i + 1) / ((float)dd / 2)) * (d_between_hooks / M_PI) : (M_PI - std::asin((dd - i - 1) / (float)dd * 2)) * (d_between_hooks / M_PI);

			Eigen::Vector3f t_t_hoot(0.112, -d_between_hooks / 2 + h_s, -0.053);
			Eigen::Vector3f l_t = hook_rotation * (p + t_t_hoot) + hook_translation;

			pcl::PointXYZ point;
			point.x = l_t[0];
			point.y = l_t[1];
			point.z = l_t[2];

			++num_points;
			cloud->points.push_back(point);
		}
	}
	
	//sphere
	//for (size_t j = 1; j <= div; j++)
	//{
	//	float z = 2. * r * j / div - r;
	//	
	//	for (size_t i = 1; i <= div; i++)
	//	{
	//		float alpha = 2. * M_PI * i / div;
	//		float rPrime = sqrt(r * r - z * z);
	//		float x = rPrime * sin(alpha);
	//		float y = rPrime * cos(alpha);

	//		

	//		pcl::PointXYZ point;
	//		point.x = x;
	//		point.y = y;
	//		point.z = z;
	//		++num_points;
	//		cloud->points.push_back(point);

	//	}
	//	
	//}
	

	CylinderModel model;
	model.belowPoint=Eigen::Vector3f(2., 2., 2.);
	model.abovePoint = Eigen::Vector3f(3., 4., 5.);
	model.div = 128;
	model.h = 2.0f;
	model.r = 1.0f;
	model.vane_count = 5;
	model.vane_width = 0.3;
	model.vane_div = 64;
	int num = drawCylinder(cloud.get(), model,true);
	num_points += num;

	// Fill in the cloud data  
	cloud->width = num_points;;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	//PCL Visualizer
	// Viewer  
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(cloud);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem();
	viewer.spin();
	return 0;
}