//
// Created by Daniel Schatz on 03.11.25.
//

#ifndef PLUSSIM_UTILITIES_H
#define PLUSSIM_UTILITIES_H

#if defined(__APPLE__)
#define scs_force_inline inline
#else
#define scs_force_inline __forceinline
#endif

void freeArray(double *&data);
void freeArray(int *&data);

#endif //PLUSSIM_UTILITIES_H