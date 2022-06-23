#ifndef DisparityMap_H
#define DisparityMap_H

#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <bitset>
#include <cstdio>

enum EndianType {
    BIG,
    LITTLE,
    ERROR
};

class DisparityMap {
private:
    int size;
    float scaleFactor; //also called endianness
    float* disparityMap;
    bool isSwapNeeded() const;
    void swapValues();

public:
    DisparityMap();
    void readMapFromFile(const std::string& filename);
    int getSize() const;
    float * getDisparityMap();
};

#endif //DisparityMap_H