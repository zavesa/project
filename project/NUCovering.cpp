#include "fragmentation.h"
#include <locale.h>
#include <cilk/cilk_api.h>


int main()
{
	setlocale(LC_ALL, "Rus");

	__cilkrts_end_cilk(); __cilkrts_set_param("nworkers", "8");
	high_level_analysis main_object; // создание объекта класса

	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	main_object.GetSolution(); // получение решения
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time = (t2 - t1);


	cout << "Тончость: " << g_precision << endl; // вывод точности
	cout << "Время выполнения: " << time.count() << " seconds" << endl; // вывод времени выполения
	cout << "Number of workers " << __cilkrts_get_nworkers() << endl;
	
	const char* out_files[3] = { "solution.txt", "boundary.txt", "not_solution.txt" };
	WriteResults(out_files); // запись результата в файлы

	return 0;
}