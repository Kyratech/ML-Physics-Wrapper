#include "include/HeightfieldData.h"

//Arbitrary flat heightmap. 32x32 picked for small size but not-awful resolution.
HeightfieldData::HeightfieldData()
{
	int defaultDimension = 32;
	this->data = new std::vector<float>(defaultDimension * defaultDimension, 0);
	this->width = defaultDimension;
	this->depth = defaultDimension;
}

HeightfieldData::HeightfieldData(std::vector<float>* mydata, int mywidth, int myheight)
{
	this->data = mydata;
	this->width = mywidth;
	this->depth = myheight;
}

HeightfieldData::HeightfieldData(const HeightfieldData& other)
{
	this->data = other.getData();
	this->width = other.getWidth();
	this->depth = other.getDepth();
}

 std::vector<float>* HeightfieldData::getData() const
{
	return data;
}

unsigned HeightfieldData::getWidth() const 
{
	return width;
}

unsigned HeightfieldData::getDepth() const 
{
	return depth;
}

void HeightfieldData::deleteData()
{
	delete data;
}