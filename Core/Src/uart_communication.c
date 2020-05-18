/*
 * uart_communication.c
 *
 *  Created on: May 17, 2020
 *      Author: iwai
 */

#include "main.h"
#include <stdio.h>
#include "fm_algorism.h"
#include "com_command.h"

extern UART_HandleTypeDef huart1;


struct ocirator_param_com param1,param2,param3,param4;


void make_algorism_param(struct note_param_com i2c_note, struct ocirator_param_com i2c_param, struct algorism_param_4op* alg_param) {
    alg_param->amp = (float)((float)i2c_param.amp100 / 100.);
    alg_param->helz_mul = i2c_param.mul;
    alg_param->helz = i2c_note.helz;
    alg_param->attack = i2c_param.attack;
    alg_param->decay = i2c_param.decay;
    alg_param->sustain_level = (float)((float)i2c_param.sus_level100 / 100.);
    alg_param->release = (float)((float)i2c_param.release_level100 / 100.);
    alg_param->muled_helz = alg_param->helz_mul * alg_param->helz ;
}

void music_data_receive()
{
    char ACK = 1;
    char NACK = 0;

    struct algorism_param_4op alg_param1, alg_param2, alg_param3, alg_param4;
    struct note_param_com note_com;
    struct command_params note_params;

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while(1) {
        // onkai(NULL);
    	HAL_StatusTypeDef st = HAL_UART_Receive(&huart1, data, RW_LENGTH, 1000);

        if(st != HAL_OK) continue;

        switch(data[0]) {
            case command_param1:
                 printf("\n");
                 printf("Received1, ");
                memcpy(&param1, &data[2], sizeof(struct ocirator_param_com));
                 printf("%d, ", param1.amp100);
                 printf("%d, ", param1.mul);
                 printf("%d, ", param1.helz);
                 printf("%d, ", param1.attack);
                 printf("%d, ", param1.decay);
                 printf("%d, ", param1.sus_level100);
                 printf("%d, ", param1.release_level100);

                HAL_UART_Transmit(&huart1, &ACK, 1, 0xFFFF);
                break;

            case command_param2:
                 printf("\n");
                 printf("Received2");
                memcpy(&param2, &data[2], sizeof(struct ocirator_param_com));
                HAL_UART_Transmit(&huart1, &ACK, 1, 0xFFFF);
                break;

            case command_param3:
                 printf("\n");
                 printf("Received3");
                memcpy(&param3, &data[2], sizeof(struct ocirator_param_com));
                HAL_UART_Transmit(&huart1, &ACK, 1, 0xFFFF);
                break;

            case command_param4:
                 printf("\n");
                 printf("Received4");
                memcpy(&param4, &data[2], sizeof(struct ocirator_param_com));
                HAL_UART_Transmit(&huart1, &ACK, 1, 0xFFFF);
                break;

            case command_note_on:
            case command_note_off:
                 printf("\n");
                 printf("Received16, ");
                memcpy(&note_com, &data[2], sizeof(struct note_param_com));
                 printf("\n");
                 printf("%d, ", note_com.on_off);
                 printf("%d, ", note_com.algorism);
                 printf("%d, ", note_com.tempo);
                 printf("%d, ", note_com.helz);
                 printf("%d, ", note_com.note_type);
                 printf("\n");

                make_algorism_param(note_com, param1, &alg_param1);
                make_algorism_param(note_com, param2, &alg_param2);
                make_algorism_param(note_com, param3, &alg_param3);
                make_algorism_param(note_com, param4, &alg_param4);

                algolism_4op algorism;
                switch(note_com.algorism) {
                    case 0:
                        algorism = YM2203_algolism0;
                        break;
                    case 1:
                        algorism = YM2203_algolism1;
                        break;
                    case 2:
                        algorism = YM2203_algolism2;
                        break;
                    case 3:
                        algorism = YM2203_algolism3;
                        break;
                    case 4:
                        algorism = YM2203_algolism4;
                        break;
                    case 5:
                        algorism = YM2203_algolism5;
                        break;
                    case 6:
                        algorism = YM2203_algolism6;
                        break;
                    case 7:
                        algorism = YM2203_algolism7;
                        break;
                    default:
                        algorism = YM2203_algolism0;
                        break;
                }

                bool success = false;

                note_params.algorism = algorism;
                note_params.note_on = note_com.on_off;
                note_params.tempo = note_com.tempo;
                note_params.note = note_com.note_type;
                note_params.param1 = &alg_param1;
                note_params.param2 = &alg_param2;
                note_params.param3 = &alg_param3;
                note_params.param4 = &alg_param4;

                success = note_4op(&note_params);
                if(success == true) {
                    HAL_UART_Transmit(&huart1, &ACK, 1, 0xFFFF);
                }
                else {
                    HAL_UART_Transmit(&huart1, &NACK, 1, 0xFFFF);
                }
                break;
        }
    }
}
