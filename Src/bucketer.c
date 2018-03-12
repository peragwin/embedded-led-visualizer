
#include "bucketer.h"
#include <stdlib.h>

static float32_t to_log_scale(float32_t val) {
    return log2(1 + val);
}

static float32_t from_log_scale(float32_t val) {
    return exp2(val) - 1;
}

Bucketer_TypeDef* NewBucketer(uint16_t size, uint16_t buckets, float32_t f_min, float32_t f_max) {
    uint16_t *indices = (uint16_t*)malloc((buckets-1) * sizeof(uint16_t));
    float32_t s_min = to_log_scale(f_min);
    float32_t s_max = to_log_scale(f_max);
    uint16_t last_index = 0;

    float32_t space = (s_max - s_min) / (float32_t)buckets;
    float32_t adj_space;
    // in scale space, how far is a unit of index
    float32_t offset_delta = (s_max - s_min) / (float32_t)size;
    float32_t offset = 1;

    float32_t v;
    uint16_t idx;
    for (int i = 0; i < buckets - 1; i++) {
        // the bucket space needs adjustment if we've accumulated offset
        adj_space = space - offset_delta * offset / (float32_t)buckets;

        v = from_log_scale((float32_t)(i+1)*adj_space + s_min + offset_delta*offset);
        idx = (uint16_t)(ceil((float32_t)size * v / f_max));

        // increment the offset if the next index hasn't incremented by at least 1;
        if (idx <= last_index) {
            idx = last_index + 1;
            offset++;
        }

        if (idx >= size) {
            idx = size - 1;
        }

        indices[i] = idx;
        last_index = idx;
    }

    float32_t *output = (float32_t*)malloc(buckets * sizeof(float32_t));
    Bucketer_TypeDef *b = (Bucketer_TypeDef*)malloc(sizeof(Bucketer_TypeDef));
    b->size = size;
    b->buckets = buckets;
    b->indices = indices;
    b->output = output;
    return b;
}

void Bucket(Bucketer_TypeDef *b, float32_t *frame) {
    uint16_t start, stop;
    float32_t sum;
    for (int i = 0; i < b->buckets; i++) {
        start = (i == 0) ? 0 : b->indices[i-1];
        stop = (i == b->buckets - 1) ? b->size : b->indices[i];
        sum = 0;
        for (int j = start; j < stop; j++) {
            sum += frame[j]; 
        }
        b->output[i] = sum / (float32_t)(stop - start);
    }
}
