#pragma once

///随机采样一致性算法应用
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