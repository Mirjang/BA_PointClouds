#pragma once
#include <mutex>
#include <condition_variable>


//SRSLY C++ Y U NO SEMAPHORE
class Semaphore
{
private:
	std::mutex mutex;
	std::condition_variable condition;
	unsigned long count = 0; // Initialized as locked.

public:
	Semaphore(const unsigned long& count) : count(count)
	{
	}

	void release() {
		std::unique_lock<std::mutex> lock(mutex);
		++count;
		condition.notify_one();
		
	}

	void aquire() {
		std::unique_lock<std::mutex> lock(mutex);
		while (!count) // Handle spurious wake-ups.
			condition.wait(lock);
		--count;
	}

	bool tryAquire() {
		std::unique_lock<std::mutex> lock(mutex);
		if (count) {
			--count;
			return true;
		}
		return false;
	}
};