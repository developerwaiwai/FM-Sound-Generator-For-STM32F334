/*
 * fm_algorism.h
 *
 *  Created on: May 14, 2020
 *      Author: iwai
 */
#include <stdbool.h>

#ifndef INC_FM_ALGORISM_H_
#define INC_FM_ALGORISM_H_

#define defaultTempo 120

/*
 * エンベロープ実行用内部変数
 */
struct internal_params {
    uint64_t atack_start_time;
    uint64_t decay_start_time;
    uint64_t release_start_time;
    float	 envepope_amp_internal;
    float	 atack_change_rate;
    float	 decay_change_rate;
};


/*
 * アルゴリズム実装パラメータ
 */
struct algorism_param_4op {
    float       amp;
    uint32_t    helz_mul;
    uint32_t    helz;
    uint32_t    muled_helz;

    uint64_t    attack;
    uint64_t    decay;
    float       sustain_level;
    float       release;

    struct internal_params internal;
};


/*
 * アルゴリズム実装定義
 */
typedef float (*algorism)(uint32_t helz);
typedef float (*algolism_4op)(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);


/*
 * コマンド実行用パラメータ
 */
struct note_param {
    algolism_4op playing_algorism;
    float		 velocity;
    struct algorism_param_4op param1, param2, param3, param4;

    bool        note_on_off;
    uint64_t    note_length;

    uint64_t    start;
    bool        start_flag;
};


/*
 * スタックオーバーフロー対策よう関数パラメータまとめ
 */
struct command_params {
	algolism_4op 				algorism;
	bool						note_on;
	uint16_t					tempo;
	uint64_t					note;
	uint8_t						velocity;
	struct algorism_param_4op* 	param1;
	struct algorism_param_4op* 	param2;
	struct algorism_param_4op* 	param3;
	struct algorism_param_4op* 	param4;
};

/*
 * アルゴリズム
 */
float YM2203_algolism0(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);
float YM2203_algolism1(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);
float YM2203_algolism2(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);
float YM2203_algolism3(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);
float YM2203_algolism4(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);
float YM2203_algolism5(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);
float YM2203_algolism6(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);
float YM2203_algolism7(float velocity, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4, uint64_t t);


/*
 * 計算処理実装
 */
inline float calc_sin(uint32_t helz);
inline float calc_sin_float(float helz, float mod, uint64_t now_time_in_us);


/*
 * 音司令
 */
void note_4op_for_mainloop(algolism_4op algorism, bool note_on, uint16_t tempo, uint64_t note, struct algorism_param_4op *param1, struct algorism_param_4op *param2, struct algorism_param_4op *param3, struct algorism_param_4op *param4);
bool note_4op(struct command_params* params);
void fm_parameter_initilalize();


/*
 * テストロジック
 */
void cho_cho_for_8algor();
void cho_cho_for_8algor_command();

#endif /* INC_FM_ALGORISM_H_ */
