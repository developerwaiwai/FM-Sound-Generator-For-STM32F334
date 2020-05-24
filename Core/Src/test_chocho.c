/*
 * test_chocho.c
 *
 *  Created on: May 15, 2020
 *      Author: iwai
 */
#include <stdio.h>
#include "fm_algorism.h"

#define whole_note 2000000
#define half_note 1000000
#define quoater_note 500000
#define eighth_note 250000


void clear_param(struct algorism_param_4op *param1){

    param1->internal.atack_start_time = param1->internal.decay_start_time = param1->internal.release_start_time = 0;
    param1->attack = 1; param1->decay = 1000000; param1->sustain_level = 0.1f;

}

//void cho_cho_for_8algor()
//{
//	struct algorism_param_4op param1;
//	struct algorism_param_4op param2;
//	struct algorism_param_4op param3;
//	struct algorism_param_4op param4;
//
//	param1.amp = 1.0f;
//	param2.amp = 1.0f;
//	param3.amp = 3.0f;
//	param4.amp = 1.0f;
//
//	param1.helz_mul = 1;
//	param2.helz_mul = 2;
//	param3.helz_mul = 3;
//	param4.helz_mul = 4;
//
//	algolism_4op algorism_[8];
//	algorism_[0] = YM2203_algolism0;
//	algorism_[1] = YM2203_algolism1;
//	algorism_[2] = YM2203_algolism2;
//	algorism_[3] = YM2203_algolism3;
//	algorism_[4] = YM2203_algolism4;
//	algorism_[5] = YM2203_algolism5;
//	algorism_[6] = YM2203_algolism6;
//	algorism_[7] = YM2203_algolism7;
//
//	for (int i = 0 ; i < 8 ; ++i) {
//		algolism_4op algorism = algorism_[i];
//
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 392;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		param1.helz = param2.helz = param3.helz = param4.helz = 330;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, half_note,     &param1, &param2, &param3, &param4);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 349;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		param1.helz = param2.helz = param3.helz = param4.helz = 294;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, half_note,     &param1, &param2, &param3, &param4);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 262;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		param1.helz = param2.helz = param3.helz = param4.helz = 294;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		param1.helz = param2.helz = param3.helz = param4.helz = 330;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		param1.helz = param2.helz = param3.helz = param4.helz = 349;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 392;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		note_4op_for_mainloop(algorism, true, defaultTempo, half_note,     &param1, &param2, &param3, &param4);
//
//	}
//
//
//	param1.amp = 0.0;
//	param2.amp = 0.0;
//	param3.amp = 0.0;
//	param4.amp = 1.0;
//
//	param4.helz_mul = 1;
//
//
//	param1.helz = param2.helz = param3.helz = param4.helz = 392;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	param1.helz = param2.helz = param3.helz = param4.helz = 330;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, half_note,     &param1, &param2, &param3, &param4);
//
//	param1.helz = param2.helz = param3.helz = param4.helz = 349;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	param1.helz = param2.helz = param3.helz = param4.helz = 294;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, half_note,     &param1, &param2, &param3, &param4);
//
//	param1.helz = param2.helz = param3.helz = param4.helz = 262;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	param1.helz = param2.helz = param3.helz = param4.helz = 294;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	param1.helz = param2.helz = param3.helz = param4.helz = 330;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	param1.helz = param2.helz = param3.helz = param4.helz = 349;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//
//	param1.helz = param2.helz = param3.helz = param4.helz = 392;
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, quoater_note,  &param1, &param2, &param3, &param4);
//	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//	note_4op_for_mainloop(YM2203_algolism7, true, defaultTempo, half_note,     &param1, &param2, &param3, &param4);
//
//}



void cho_cho_for_8algor_command()
{
	struct algorism_param_4op param1;
	struct algorism_param_4op param2;
	struct algorism_param_4op param3;
	struct algorism_param_4op param4;

	param1.amp = 1.0f;
	param2.amp = 1.0f;
	param3.amp = 3.0f;
	param4.amp = 1.0f;

	param1.helz_mul = 1;
	param2.helz_mul = 2;
	param3.helz_mul = 3;
	param4.helz_mul = 4;

	algolism_4op algorism_[8];
	algorism_[0] = YM2203_algolism0;
	algorism_[1] = YM2203_algolism1;
	algorism_[2] = YM2203_algolism2;
	algorism_[3] = YM2203_algolism3;
	algorism_[4] = YM2203_algolism4;
	algorism_[5] = YM2203_algolism5;
	algorism_[6] = YM2203_algolism6;
	algorism_[7] = YM2203_algolism7;

	struct command_params	params = {0};
	params.param1 = &param1;
	params.param2 = &param2;
	params.param3 = &param3;
	params.param4 = &param4;

	params.velocity = 100;

//	for (int i = 0 ; i < 8 ; ++i) {
//		algolism_4op algorism = algorism_[i];
//
//		params.velocity = 127;
//
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 392;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 330;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = half_note;
//		note_4op_delaying(&params);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 349;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 294;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = half_note;
//		note_4op_delaying(&params);
//
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 262;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 294;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 330;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 349;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		param1.helz = param2.helz = param3.helz = param4.helz = 392;
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = quoater_note;
//		note_4op_delaying(&params);
//
//		clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
//		params.algorism = algorism;
//		params.note_on = true;
//		params.tempo = defaultTempo;
//		params.note = half_note;
//		note_4op_delaying(&params);
//
//	}


	param1.amp = 0.0;
	param2.amp = 0.0;
	param3.amp = 0.0;
	param4.amp = 1.0;

	param4.helz_mul = 1;


	param1.helz = param2.helz = param3.helz = param4.helz = 392;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 330;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = half_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 349;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 294;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = half_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 262;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 294;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 330;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 349;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	param1.helz = param2.helz = param3.helz = param4.helz = 392;
	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = quoater_note;
	note_4op_delaying(&params);

	clear_param(&param1); clear_param(&param2); clear_param(&param3); clear_param(&param4);
	params.algorism = YM2203_algolism7;
	params.note_on = true;
	params.tempo = defaultTempo;
	params.note = half_note;
	note_4op_delaying(&params);

}

