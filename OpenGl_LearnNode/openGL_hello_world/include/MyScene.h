#ifndef MYSCENE_H
#define MYSCENE_H

#include <memory>
#include <string>

class MyScene
{
public:
	MyScene(const std::string& windowTitle, int width, int height);

	void run();

	~MyScene();
private:
	class PImpl;
	std::unique_ptr<PImpl> impl_;
};

#endif // MYSCENE_H
