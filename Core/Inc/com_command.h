/*
 * com_command.h
 *
 *  Created on: May 17, 2020
 *      Author: iwai
 */

#ifndef INC_COM_COMMAND_H_
#define INC_COM_COMMAND_H_

/*
 * 通信用定義と構造体
 */
#define BUF_SIZE (1024)

#define command_param1      0x01
#define command_param2      0x02
#define command_param3      0x03
#define command_param4      0x04

#define command_note_on     0x10
#define command_note_off    0x11

#define RW_LENGTH 34               /*!< Data length for r/w test, [0,DATA_LENGTH] */

struct ocirator_param_com {
    uint16_t    amp100;
    uint16_t    mul;
    uint32_t    helz;

    uint64_t    attack;
    uint64_t    decay;
    uint16_t    sus_level100;
    uint16_t    release_level100;
};

struct note_param_com {
    uint16_t    on_off;
    uint16_t    algorism;
    uint16_t    tempo;
    uint16_t    helz;
    uint64_t    note_type;
    uint64_t	velocity;
    uint64_t    reserved;
};


void music_data_receive();


#endif /* INC_COM_COMMAND_H_ */
