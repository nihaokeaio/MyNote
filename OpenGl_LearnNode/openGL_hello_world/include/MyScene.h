#ifndef MYSCENE_H
#define MYSCENE_H

#include <memory>

class MyScene
{
public:
	MyScene();

	~MyScene();
private:
	class PImpl;
	std::unique_ptr<PImpl> impl_;
};

#endif // MYSCENE_H
