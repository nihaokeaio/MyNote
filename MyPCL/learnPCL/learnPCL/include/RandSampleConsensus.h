#pragma once

///�������һ�����㷨Ӧ��
#include "commonInclude.h"

class ShowDebug;


class RandSampleConsensus
{
public:
	RandSampleConsensus();

	void demoSphere();

	void demoPlane();
private:
	std::shared_ptr<ShowDebug> viewer_;
};