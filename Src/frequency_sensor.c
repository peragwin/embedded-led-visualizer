#include "frequency_sensor.h"
#include "fastLog.h"
#include <stdlib.h>

FilterValues_TypeDef* NewFilterValues(uint16_t size, float32_t *gain_values, float32_t *diff_values) {
    float32_t *gain = (float32_t*)malloc(size * sizeof(float32_t));
    if (gain_values != NULL) {
        memcpy(gain, gain_values, size * sizeof(float32_t));
    }
    float32_t* diff = (float32_t*)malloc(size *sizeof(float32_t));
    if (diff_values != NULL) {
        memcpy(diff, diff_values, size * sizeof(float32_t));
    }
    FilterValues_TypeDef *fv = (FilterValues_TypeDef*)malloc(sizeof(FilterValues_TypeDef));
    fv->gain = gain;
    fv->diff = diff;
    fv->size = size;
    return fv;
}

Drivers_TypeDef* NewDrivers(uint16_t size, uint16_t columns) {
    float32_t **amp = (float32_t**)malloc(columns * sizeof(float32_t*));
    for (int i = 0; i < columns; i++) {
        float32_t *amp_vals = (float32_t*)calloc(size, sizeof(float32_t));
        amp[i] = amp_vals;
    }
    float32_t *diff = (float32_t*)calloc(size, sizeof(float32_t));
    float32_t *energy = (float32_t*)calloc(size, sizeof(float32_t));
    Drivers_TypeDef *d = (Drivers_TypeDef*)malloc(sizeof(Drivers_TypeDef));
    d->amp = amp;
    d->diff = diff;
    d->energy = energy;
    d->bass = 0;
    return d;
}

GainController_TypeDef* NewGainController(uint16_t size, float32_t *params, float32_t kp, float32_t kd) {
    float32_t *filterParams = (float32_t*)malloc(2 * sizeof(float32_t));
    memcpy(filterParams, params, 2*sizeof(float32_t));
    float32_t *gain = (float32_t*)calloc(size, sizeof(float32_t));
    float32_t *frame = (float32_t*)calloc(size, sizeof(float32_t));
    float32_t *err = (float32_t*)calloc(size, sizeof(float32_t));
    GainController_TypeDef* gc = (GainController_TypeDef*)malloc(sizeof(GainController_TypeDef));
    gc->frame_size = size;
    gc->filterParams = filterParams;
    gc->gain = gain;
    gc->frame = frame;
    gc->err = err;
    gc->kp = kp;
    gc->kd = kd;
    return gc;
}

FrequencySensor_TypeDef* NewFrequencySensor(uint16_t size, uint16_t columns) {
    Parameters_TypeDef params = {
        .offset = 0,
        .gain = 2,
        .differential_gain = 2e-3,
        .sync = 1e-2,
    };
    float32_t gainFilterParams[4] = {
        0.80, 0.20,
        -0.005, 0.995,
    };
    float32_t diffFilterParams[4] = {
        0.263, .737,
        -0.0028, 0.2272,
    };
    float32_t preemphasis = 16;
    float32_t gainControllerParams[2] = {0.05, 0.95};
    float32_t kp = 0.005;
    float32_t kd = 0.001;
    
    Parameters_TypeDef *paramValues = (Parameters_TypeDef*)malloc(sizeof(Parameters_TypeDef));
    memcpy(paramValues, &params, sizeof(Parameters_TypeDef));
    FilterValues_TypeDef *filterParams = NewFilterValues(4, gainFilterParams, diffFilterParams);
    FilterValues_TypeDef *filterValues = NewFilterValues(2 * size, NULL, NULL);
    GainController_TypeDef *agc = NewGainController(size, gainControllerParams, kp, kd);
    Drivers_TypeDef *drivers = NewDrivers(size, columns);

    FrequencySensor_TypeDef *fs = (FrequencySensor_TypeDef*)malloc(sizeof(FrequencySensor_TypeDef));
    fs->size = size;
    fs->columns = columns;
    fs->params = paramValues;
    fs->filterParams = filterParams;
    fs->filterValues = filterValues;
    fs->agc = agc;
    fs->drivers = drivers;
    fs->preemphasis = preemphasis;
    fs->render_lock = 0;
    return fs;
}

void apply_premphasis(FrequencySensor_TypeDef *fs, float32_t *frame) {
    float32_t incr = (fs->preemphasis - 1) / (float32_t)fs->size;
    for (int i = 0; i < fs->size; i++) {
        frame[i] *= 1 + (float32_t)i*incr;
    }
}

// use matrix multiplication to apply a two tap IIR filter across an entire frame
void mat_apply_filter(float32_t *params, float32_t *in, float32_t *out, uint16_t size) {
    float32_t m[size * 2];
    float32_t mt[size * 2];
    memcpy(m, in, size * sizeof(float32_t));
    memcpy(m+size, out, size * sizeof(float32_t));
    arm_matrix_instance_f32 M = {
        .numCols = size,
        .numRows = 2,
        .pData = m,
    };
    arm_matrix_instance_f32 M_T = {
        .numCols = 2,
        .numRows = size,
        .pData = mt,
    };
    arm_mat_trans_f32(&M, &M_T);
    // TODO: initialize filterParams as a matrix
    arm_matrix_instance_f32 a = {
        .numCols = 1,
        .numRows = 2,
        .pData = params,
    };
    arm_matrix_instance_f32 b = {
        .numCols = 1,
        .numRows = size,
        .pData = out,
    };
    if (arm_mat_mult_f32(&M_T, &a, &b))
        Error_Handler();
}

static float32_t quadratic_error(float32_t x) {
    float32_t sign = (x < 0) ? -1.0 : 1.0;
    return sign * x * x;
}

static float32_t log_error(float32_t x) {
    x = 1.000001 - x;
    float32_t sign = (x < 0) ? 1.0 : -1.0;
    float32_t a = (x < 0) ? -x : x;
    return sign * log2_2521(x);
}

void apply_gain_control(FrequencySensor_TypeDef *fs, float32_t *frame) {
    // apply gain
    arm_mult_f32(frame, fs->agc->gain, frame, fs->size);

    // apply filter
    mat_apply_filter(fs->agc->filterParams, frame, fs->agc->frame, fs->size);

    // calculate error
    float32_t e[fs->size];
    for (int i = 0; i < fs->size; i++) {
        e[i] = log_error(1 - fs->agc->frame[i]);
    }
    
    // apply pd controller
    float32_t u;
    float32_t kp = fs->agc->kp;
    float32_t kd = fs->agc->kd;
    for (int i = 0; i < fs->size; i++) {
        float32_t gain = fs->agc->gain[i];
        float32_t err = fs->agc->err[i];
        u = kp * e[i] + kd * (e[i] - err);
        gain += u;
        if (gain > 10000) gain = 10000;
        if (gain < .001) gain = .001;
        fs->agc->gain[i] = gain;
        fs->agc->err[i] = e[i];
    }
}

void apply_filter(FrequencySensor_TypeDef *fs, float32_t *frame, float32_t *out, float32_t *params, float32_t *diff_output) {
    for (int level = 0; level < 2; level++) {
        mat_apply_filter(params, frame, out, fs->size);

        if (diff_output != NULL) {
            arm_add_f32(frame, out, diff_output, fs->size);
        }

        memcpy(out + level*fs->size, frame, fs->size * sizeof(float32_t));
    }

    // apply feedback
    arm_add_f32(out, out+fs->size, out, fs->size);
}

void apply_filters(FrequencySensor_TypeDef *fs, float32_t *frame) {
    float32_t diff_output[fs->size];
    apply_filter(fs, frame, fs->filterValues->gain, fs->filterParams->gain, diff_output);
    apply_filter(fs, diff_output, fs->filterValues->diff, fs->filterParams->diff, NULL);
}

void apply_effects(FrequencySensor_TypeDef *fs) {
    float32_t dg = fs->params->differential_gain;
    float32_t ag = fs->params->gain;
    float32_t ao = fs->params->offset;
    float32_t *gain = fs->filterValues->gain;
    float32_t *diff = fs->filterValues->diff;

    // apply column animation
    if (fs->params->mode == 1) {
        float32_t decay = 1.0 - (2.0 / (float32_t)fs->columns);
        for (int i = fs->columns / 2; i >= 0; i--) {
            memcpy(fs->drivers->amp[i+1], fs->drivers->amp[i], fs->size * sizeof(float32_t));
            for (int j = 0; j < fs->size; j++) {
                fs->drivers->amp[i+1][j] *= decay;
            }
        }
    }

    float32_t *amp = fs->drivers->amp[0];
    float32_t *fs_diff = fs->drivers->diff;
    float32_t *energy = fs->drivers->energy;
    float32_t ph;
    memcpy(amp, gain, fs->size * sizeof(float32_t));
    memcpy(fs_diff, diff, fs->size * sizeof(float32_t));
    for (int i = 0; i < fs->size; i++) {
        amp[i] = ao + ag * amp[i];
        ph = energy[i] + .001;
        ph -= dg * (diff[i] < 0 ? -diff[i] : diff[i]);
        energy[i] = ph;
    }
}

void apply_sync(FrequencySensor_TypeDef *fs) {
    float32_t *energy = fs->drivers->energy;
    float32_t mean = 0;
    for (int i = 0; i < fs->size; i++) {
        mean += energy[i];
    }
    mean /= (float32_t)fs->size;
    if (mean < -2*PI) {
        for (int i = 0; i < fs->size; i++) {
            energy[i] = 2*PI + fmod(energy[i], 2*PI);
        }
        mean = 2*PI + fmod(mean, 2*PI);
    }
    if (mean > 2*PI) {
        for (int i = 0; i < fs->size; i++) {
            energy[i] = fmod(energy[i], 2*PI);
        }
        mean = fmod(mean, 2*PI);
    }
    float32_t diff;
    float32_t sign;
    for (int i = 0; i < fs->size; i++) {
        diff = mean - energy[i];
        sign = (diff < 0) ? -1.0 : 1.0;
        diff = sign * diff * diff;
        energy[i] += fs->params->sync * diff;
    }
}

void apply_base(FrequencySensor_TypeDef *fs) {
    float32_t *diff = fs->drivers->diff;
    float32_t bass = 0;
    float32_t scale, v;
    for (int i = 0; i < 4; i++) {
        scale = (4.0 - (float32_t)i) / 4.0;
        v = diff[i];
        if (v < 0) v = 0;
        bass += v * scale;
    }
    bass /= 2.0;
    bass = log2_2521(1 + bass);
    fs->drivers->bass = .75 * bass + .25 * fs->drivers->bass;
}

void FS_Process(FrequencySensor_TypeDef *fs, float32_t *frame) {
    apply_premphasis(fs, frame);
    apply_gain_control(fs, frame);
    apply_filters(fs, frame);
    apply_effects(fs);
    apply_sync(fs);
    apply_base(fs);
}
