#pragma once
#include <chrono>

class Timer
{
public:
	std::chrono::time_point<std::chrono::steady_clock>start, end;
	std::chrono::duration<float>duration;

	Timer()
	{
		start = std::chrono::high_resolution_clock::now();
	}

	float stop(bool bReset = false)
	{
		end = std::chrono::high_resolution_clock::now();
		duration = end - start;
		float ms = duration.count() * 1000.0f;
		if (bReset)
		{
			start = end;
		}
		return ms;
	}

	~Timer()
	{
		stop();
	}
};