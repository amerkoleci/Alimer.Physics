// Copyright (c) Amer Koleci and Contributors.
// Licensed under the MIT License (MIT). See LICENSE in the repository root for more information.

#include "alimer_internal.h"

void* alimerCalloc(size_t count, size_t size)
{
    return calloc(count, size);
}

void* alimerMalloc(size_t size)
{
    return malloc(size);
}

void* alimerRealloc(void* old, size_t size)
{
    void* data = realloc(old, size);
    if (!data) abort();
    //alimerProfileAlloc(data, size);
    return data;
}

void alimerFree(void* data)
{
    free(data);
}
