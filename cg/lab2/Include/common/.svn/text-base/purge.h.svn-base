#pragma once

struct deleter {
	template <class T> void operator () (T * p) { delete p; }
};

template <class C>
	void purge(C &container)
{
	std::for_each(container.begin(), container.end(), deleter());
	container.clear();
}

