#pragma once

/// ��������� Box ��������� ���������� ��������� box-�
struct Box
{
protected:
	/// ����� ���������� x box-a
	double x_min;
	/// ������ ���������� y box-a
	double y_min;
	/// ������ box-a
	double width;
	/// ������ box-a
	double height;
public:
	Box() {}

	Box(double min_x, double min_y, double x_width, double y_height);

	~Box() {}

	/// ������� GetParameters() ���������� ���������� ��������� box-�
	void GetParameters(double& min_x, double& min_y, double& x_width, double& y_height) const;

	/// ������� GetWidhtHeight() ���������� ���������� �������� ������ � ������ box-a
	void GetWidhtHeight(double& x_width, double& y_height) const;

	/// ������� GetDiagonal() ���������� ��������� box-a
	double GetDiagonal() const;
};