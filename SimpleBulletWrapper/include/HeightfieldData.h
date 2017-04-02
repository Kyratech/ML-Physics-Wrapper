#ifndef HEIGHTFIELD_DATA_WRAPPER
#define HEIGHTFIELD_DATA_WRAPPER

#include <vector>

class HeightfieldData
{
public:
	HeightfieldData();
	HeightfieldData(std::vector<float>* mydata, int mywidth, int myheight);
	HeightfieldData(const HeightfieldData& other);

	void deleteData();

	std::vector<float>* getData() const;
	unsigned getWidth() const;
	unsigned getDepth() const;

private:
	std::vector<float>* data;
	int width;
	int depth;
};

#endif