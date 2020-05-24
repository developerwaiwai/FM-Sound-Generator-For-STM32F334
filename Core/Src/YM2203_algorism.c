/*
 * YM2203_algorism.c
 *
 *  Created on: May 14, 2020
 *      Author: iwai
 */

#include <stdio.h>
#include "fm_algorism.h"


extern volatile uint64_t get_system_count_in_us();


float envelope(struct algorism_param_4op *param1, uint64_t t)
{

    float* amp1 = &(param1->internal.envepope_amp_internal);

    *amp1 = (param1->internal.atack_change_rate * (float)(t - param1->internal.atack_start_time));
    if (*amp1 >= 1.f) {
        *amp1 = 1.f - (param1->internal.decay_change_rate * (float)(t - param1->internal.decay_start_time));

        if (*amp1 <= param1->sustain_level) {
            *amp1 = param1->sustain_level;
        }
    }

    return *amp1;
}


float YM2203_algolism0(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, 0, t);
    float Y = amp4 * calc_sin_float((float)param4->muled_helz , amp3 * calc_sin_float((float)param3->muled_helz, amp2 * calc_sin_float((float)param2->muled_helz , amp1 * calc_sin_float((float)param1->muled_helz ,Y1, t), t), t), t);

    // for test
    // float Y = param4->amp * calc_sin_float((float)helz4, param2->amp * calc_sin_float((float)helz2, 0, t), t);

    Y = (Y) * (1.0f / (param4->amp * 2.f));
    return Y;
}


float YM2203_algolism1(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, 0, t);
    float Y2 = (amp1 * calc_sin_float((float)param1->muled_helz, Y1, t)) + (amp2 * calc_sin_float((float)param2->muled_helz, 0, t));
    float Y  = amp4 * calc_sin_float((float)param4->muled_helz, amp3 * calc_sin_float((float)param3->muled_helz, Y2, t), t);

    Y = (Y) * (1.0f / (param4->amp * 2.f));
    return Y;
}


float YM2203_algolism2(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, amp1 * calc_sin_float((float)param1->muled_helz, 0, t), t);
    float Y2 = amp3 * calc_sin_float((float)param3->muled_helz, amp2 * calc_sin_float((float)param2->muled_helz, 0, t), t);
    float Y  = amp4 * calc_sin_float((float)param4->muled_helz, Y1 + Y2, t);

    Y = (Y) * (1.0f / (param4->amp * 2.f));
    return Y;
}


float YM2203_algolism3(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, amp1 * calc_sin_float((float)param1->muled_helz, 0, t), t);
    Y1 = amp2 * calc_sin_float((float)param2->muled_helz, Y1, t);
    float Y2 = amp3 * calc_sin_float((float)param3->muled_helz, 0, t);
    float Y = amp4 * calc_sin_float((float)param4->muled_helz, Y1 + Y2, t);

    Y = (Y) * (1.0f / (param4->amp * 2.f));
    return Y;
}


float YM2203_algolism4(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, amp1 * calc_sin_float((float)param1->muled_helz, 0, t), t);
    float Y2 = amp2 * calc_sin_float((float)param2->muled_helz, Y1, t);
    float Y3 = amp4 * calc_sin_float((float)param4->muled_helz, amp3 * calc_sin_float((float)param3->muled_helz, 0, t), t);

    float Y  = Y2 + Y3;

    float amp_total = param2->amp + param4->amp;
    Y = (Y) * (1.0f / (amp_total * 2.f));
    return Y;
}


float YM2203_algolism5(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, amp1 * calc_sin_float((float)param1->muled_helz, 0, t), t);
    float Y2 = amp2 * calc_sin_float((float)param2->muled_helz, Y1, t);
    float Y3 = amp3 * calc_sin_float((float)param3->muled_helz, Y1, t);
    float Y4 = amp4 * calc_sin_float((float)param4->muled_helz, Y1, t);

    float Y = Y2+Y3+Y4;

    float amp_total = param2->amp + param3->amp + param4->amp;
    Y = (Y) * (1.0f / (amp_total * 2.f));
    return Y;
}


float YM2203_algolism6(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, amp1 * calc_sin_float((float)param1->muled_helz, 0, t), t);
    float Y2 = amp2 * calc_sin_float((float)param2->muled_helz, Y1, t);
    float Y3 = amp3 * calc_sin_float((float)param3->muled_helz, 0, t);
    float Y4 = amp4 * calc_sin_float((float)param4->muled_helz, 0, t);

    float Y = Y2+Y3+Y4;

    float amp_total = param2->amp + param3->amp + param4->amp;
    Y = (Y) * (1.0f / (amp_total * 2.f));
    return Y;
}


float YM2203_algolism7(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t)
{
    float amp1 = param1->amp * envelope(param1, t) * velocity;
    float amp2 = param2->amp * envelope(param2, t) * velocity;
    float amp3 = param3->amp * envelope(param3, t) * velocity;
    float amp4 = param4->amp * envelope(param4, t) * velocity;

    float Y1 = amp1 * calc_sin_float((float)param1->muled_helz, amp1 * calc_sin_float((float)param1->muled_helz, 0, t), t);
    float Y2 = amp2 * calc_sin_float((float)param2->muled_helz, 0, t);
    float Y3 = amp3 * calc_sin_float((float)param3->muled_helz, 0, t);
    float Y4 = amp4 * calc_sin_float((float)param4->muled_helz, 0, t);
    float Y = Y1+Y2+Y3+Y4;

    float amp_total = param1->amp + param2->amp + param3->amp + param4->amp;
    Y = (Y) * (1.0f / (amp_total * 2.f));
    return Y;
}
