//
// Created by Daniel Schatz on 03.11.25.
//

#include "utilities.h"

void freeArray(double *&data) {
    delete[] data;
    data = nullptr;
}

void freeArray(int *&data) {
    delete[] data;
    data = nullptr;
}