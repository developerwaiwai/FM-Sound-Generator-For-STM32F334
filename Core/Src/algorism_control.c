/*
 * algorism_control.c
 *
 *  Created on: May 15, 2020
 *      Author: iwai
 */
#include <stdio.h>
#include "fm_algorism.h"
#include "cQueue.h"
#include <string.h>

/* from main.c */
extern uint16_t devide;
extern float MAX_DAC_OUTPUT;
inline uint64_t get_system_count_in_us();


volatile Queue_t			command_queue;
volatile struct note_param 	param_g = {0};


void fm_parameter_initilalize()
{
	//initialize parameter
    memset(&param_g, 0 , sizeof(struct note_param));
    param_g.param1.amp = 1.;
    param_g.param1.helz_mul = 1;
    param_g.param1.helz = 440;
    param_g.param1.muled_helz = 440;
    param_g.param1.attack = 0;
    param_g.param1.decay = 0;
    param_g.param1.sustain_level = 1.;
    param_g.param1.release = 0.;
    memcpy (&(param_g.param2), &(param_g.param1), sizeof(struct algorism_param_4op));
    memcpy (&(param_g.param3), &(param_g.param1), sizeof(struct algorism_param_4op));
    memcpy (&(param_g.param4), &(param_g.param1), sizeof(struct algorism_param_4op));

    param_g.playing_algorism = YM2203_algolism0;
    param_g.note_on_off = false;
    param_g.note_length = 0;
    param_g.start = 0;
    param_g.start_flag = 0;
}

void initialize_fm_sound_controller()
{
	//initialize queue
	q_init(&command_queue, sizeof(struct note_param), 10, FIFO, false);

	//initialize parameter
	fm_parameter_initilalize();
}


void processing_in_interrupt()
{
	if(param_g.note_length > 0 && get_system_count_in_us() - param_g.start >= param_g.note_length) {
    	bool ret = q_pop(&command_queue, &param_g);
        if(ret == true) {
        	param_g.start = get_system_count_in_us();

        	param_g.param1.internal.atack_start_time = param_g.start;
        	param_g.param1.internal.decay_start_time = param_g.start + param_g.param1.attack;

        	param_g.param2.internal.atack_start_time = param_g.start;
        	param_g.param2.internal.decay_start_time = param_g.start + param_g.param2.attack;

        	param_g.param3.internal.atack_start_time = param_g.start;
        	param_g.param3.internal.decay_start_time = param_g.start + param_g.param3.attack;

        	param_g.param4.internal.atack_start_time = param_g.start;
        	param_g.param4.internal.decay_start_time = param_g.start + param_g.param4.attack;

            param_g.start_flag = true;
        }
        else{
            param_g.start = 0;
            param_g.start_flag = false;
        }
    }

	if(param_g.start == 0 && param_g.start_flag == false) {
		return;
	}
//	uint64_t s = get_system_count_in_us();

	devide = MAX_DAC_OUTPUT * (param_g.playing_algorism(&param_g.param1, &param_g.param2, &param_g.param3, &param_g.param4, get_system_count_in_us() + 50));
//	uint64_t st = get_system_count_in_us() - s;
//	printf("st = %d, ", st);
//	int i = 0;
}




void calculate_wave_4op(algolism_4op algor, bool noteOn, uint64_t time, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4)
{
    time -= 10000;

    uint64_t start = get_system_count_in_us();
    uint64_t stop = get_system_count_in_us();
	param1->internal.atack_start_time = start;
	param1->internal.decay_start_time = start + param1->attack;

	param2->internal.atack_start_time = start;
	param2->internal.decay_start_time = start + param2->attack;

	param3->internal.atack_start_time = start;
	param3->internal.decay_start_time = start + param3->attack;

	param4->internal.atack_start_time = start;
	param4->internal.decay_start_time = start + param4->attack;

	while (1) {
        stop = get_system_count_in_us();
        if(stop - start >= time) {
            uint64_t start2 = get_system_count_in_us();
            uint64_t stop2 = get_system_count_in_us();
            while(1){
                stop2 = get_system_count_in_us();
                if(stop2 - start2 >= 10000) {
                	devide = 0;
                	break;
                }
            }
            break;
        }

        if(noteOn == false) {
        	devide = 0;
            return;
        }
        else
        {
            float Y = algor(param1, param2, param3, param4, get_system_count_in_us());
            devide = MAX_DAC_OUTPUT * Y;
        }
    }
}


bool calculate_wave_4op2(algolism_4op algor, bool noteOn, uint64_t time, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4)
{

	bool ret = false;

	struct note_param param_local;

    param_local.playing_algorism = algor;
    param_local.note_on_off = noteOn;
    param_local.note_length = time;

    memcpy(&param_local.param1, param1, sizeof(struct algorism_param_4op));
    memcpy(&param_local.param2, param2, sizeof(struct algorism_param_4op));
    memcpy(&param_local.param3, param3, sizeof(struct algorism_param_4op));
    memcpy(&param_local.param4, param4, sizeof(struct algorism_param_4op));

    if(param_g.start != 0 && param_g.start_flag == true) {
    	ret = q_push(&command_queue, &param_local);
    }
    else {
        if(q_isEmpty(&command_queue) == true) {
            memcpy(&param_g, &param_local, sizeof(struct note_param));
            param_g.start = get_system_count_in_us();
            param_g.start_flag = true;

            ret = true;
        }
        else {
        	param_local.start = 0;
        	param_local.start_flag = false;
        	ret = q_push(&command_queue, &param_local);
        }
    }

    return ret;
}


void note_4op_for_mainloop(algolism_4op algorism, bool note_on, uint16_t tempo, uint64_t note, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4)
{
    uint64_t note_length = note * tempo / defaultTempo;

    param1->muled_helz = param1->helz_mul * param1->helz;
    param2->muled_helz = param2->helz_mul * param2->helz;
    param3->muled_helz = param3->helz_mul * param3->helz;
    param4->muled_helz = param4->helz_mul * param4->helz;

    param1->internal.atack_start_time = param1->internal.decay_start_time = param1->internal.release_start_time = 0;
    param2->internal.atack_start_time = param2->internal.decay_start_time = param2->internal.release_start_time = 0;
    param3->internal.atack_start_time = param3->internal.decay_start_time = param3->internal.release_start_time = 0;
    param4->internal.atack_start_time = param4->internal.decay_start_time = param4->internal.release_start_time = 0;

    param1->internal.atack_change_rate = 1.f / (float)param1->attack;
    param2->internal.atack_change_rate = 1.f / (float)param2->attack;
    param3->internal.atack_change_rate = 1.f / (float)param3->attack;
    param4->internal.atack_change_rate = 1.f / (float)param4->attack;

    param1->internal.decay_change_rate = 1.f / (float)param1->decay;
    param2->internal.decay_change_rate = 1.f / (float)param2->decay;
    param3->internal.decay_change_rate = 1.f / (float)param3->decay;
    param4->internal.decay_change_rate = 1.f / (float)param4->decay;

    calculate_wave_4op(algorism, note_on, note_length, param1, param2, param3, param4);
}


bool note_4op(struct command_params* params)
{
    uint64_t note_length = params->note *  defaultTempo / params->tempo;

    params->param1->muled_helz = params->param1->helz_mul * params->param1->helz;
    params->param2->muled_helz = params->param2->helz_mul * params->param2->helz;
    params->param3->muled_helz = params->param3->helz_mul * params->param3->helz;
    params->param4->muled_helz = params->param4->helz_mul * params->param4->helz;

    params->param1->internal.atack_start_time = params->param1->internal.decay_start_time = params->param1->internal.release_start_time = 0;
    params->param2->internal.atack_start_time = params->param2->internal.decay_start_time = params->param2->internal.release_start_time = 0;
    params->param3->internal.atack_start_time = params->param3->internal.decay_start_time = params->param3->internal.release_start_time = 0;
    params->param4->internal.atack_start_time = params->param4->internal.decay_start_time = params->param4->internal.release_start_time = 0;

    params->param1->internal.atack_change_rate = 1.f / (float)params->param1->attack;
    params->param2->internal.atack_change_rate = 1.f / (float)params->param2->attack;
    params->param3->internal.atack_change_rate = 1.f / (float)params->param3->attack;
    params->param4->internal.atack_change_rate = 1.f / (float)params->param4->attack;

    params->param1->internal.decay_change_rate = 1.f / (float)params->param1->decay;
    params->param2->internal.decay_change_rate = 1.f / (float)params->param2->decay;
    params->param3->internal.decay_change_rate = 1.f / (float)params->param3->decay;
    params->param4->internal.decay_change_rate = 1.f / (float)params->param4->decay;

    return calculate_wave_4op2(params->algorism, params->note_on, note_length, params->param1, params->param2, params->param3, params->param4);
}


note_4op_delaying(struct command_params* params)
{
	bool ret = false;

	while (ret == false) {
		ret = note_4op(params);
	}
}


