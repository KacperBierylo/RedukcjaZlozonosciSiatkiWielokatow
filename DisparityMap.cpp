#include "DisparityMap.h"

bool DisparityMap::isSwapNeeded() const {
    if (scaleFactor != 0.f) {
        uint32_t endianness = 0xdeadbeef;
        auto *temp = (unsigned char *) &endianness;
        EndianType endianType = (((*temp) ^ 0xef) == 0 ? LITTLE : (*temp) != 0 ? BIG : ERROR);

        return (BIG == endianType) && (this->scaleFactor < 0.f);
    }

    return false;
}

void DisparityMap::swapValues() {
    union {
        float f;
        unsigned char u8[sizeof(float)];
    } source{}, destination{};

    for (int j = 0; j < size; ++j) {
        source.f = disparityMap[j];
        for (int k = 0; k < sizeof(float); ++k) {
            destination.u8[k] = source.u8[sizeof(float) - k - 1];
        }

        disparityMap[j] = destination.f;
    }
}

DisparityMap::DisparityMap() {
    size = 0;
    scaleFactor = 0.f;
    disparityMap = nullptr;
}

void DisparityMap::readMapFromFile(const std::string& filename) {
    FILE * file;
    file = fopen(filename.c_str(), "rb");

    char auxiliaryArr[128];
    if (file != nullptr) {
        fscanf(file, "%s", auxiliaryArr);
        if (!strcmp(auxiliaryArr, "Pf")) {
            fscanf(file, "%s", auxiliaryArr);
            int width = atoi(auxiliaryArr);

            fscanf(file, "%s", auxiliaryArr);
            int height = atoi(auxiliaryArr);

            size = width * height;

            fscanf(file, "%s", auxiliaryArr);
            scaleFactor = static_cast<float>(atof(auxiliaryArr));

            fseek(file, 0, SEEK_END);
            long tempSize = ftell(file);
            long pos = tempSize - width * height * sizeof(float);
            fseek(file, pos, SEEK_SET);

            auto* auxiliaryMap = new float[size];
            fread(auxiliaryMap, sizeof(float), size, file);
            fclose(file);

            disparityMap = new float[size];
            for (int i = 0; i < height; ++i) {
                memcpy(&disparityMap[(height - i - 1) * width], &auxiliaryMap[i * width], width * sizeof(float));
            }

            if (this->isSwapNeeded()) {         // this block is probably to remove
                swapValues();
            }

            delete[] auxiliaryMap;
        }
    }
    else {
        throw std::runtime_error("File doesn't exists!");
    }
}

int DisparityMap::getSize() const {
    return size;
}

float *DisparityMap::getDisparityMap() {
    return disparityMap;
}