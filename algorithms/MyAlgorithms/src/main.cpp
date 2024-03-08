#include <iostream>

#include "MyAlgorithms.h"

void findStr(MyAlgorithms& myAlgorithms)
{
	string txt = "mississippi";
	string pat = "issip";

	int res = myAlgorithms.serachStrKMP(txt, pat);

	std::cout << "res = " << res << std::endl;
}




void main()
{
	MyAlgorithms myAlgorithms;

	findStr(myAlgorithms);




	system("pause");
}