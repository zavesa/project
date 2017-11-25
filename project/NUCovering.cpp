#include "fragmentation.h"
#include <locale.h>


int main()
{
	setlocale(LC_ALL, "Rus");

	high_level_analysis main_object; // �������� ������� ������
	
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	main_object.GetSolution(); // ��������� �������
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time = (t2 - t1);

	cout << "��������: " << g_precision << endl; // ����� ��������
	cout << "����� ����������: " << time.count() << " seconds" << endl; // ����� ������� ���������
	
	const char* out_files[3] = { "solution.txt", "boundary.txt", "not_solution.txt" };
	WriteResults(out_files); // ������ ���������� � �����

	return 0;
}