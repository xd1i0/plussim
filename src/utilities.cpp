//
// Created by Daniel Schatz on 03.11.25.
//

#include "utilities.h"

void atg_scs::freeArray(double *&data) {
    delete[] data;
    data = nullptr;
}

void atg_scs::freeArray(int *&data) {
    delete[] data;
    data = nullptr;
}