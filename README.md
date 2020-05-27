# FM-Sound-Generator-For-STM32F334


# What is this
This is FM sound generator on STM32F334.

This project is written by C with using STM32CubeIDE.
You can play any music with specail suquenceser. That was published on Github. Please see link bellow.

https://github.com/developerwaiwai/Sequencer-FM-sound-generator-for-STM32


# How to build
You can build and transmit to STM32 with using STM32CubeIDE.
After setup IDE and board, you can only push "build button".

The MPU wait data via USART2. When the appropriate data is received by MPU, MPU play the music.
The data are two types. First, it is for sound color, second, it is for note.
I explain this data in sequencer repository.


# sample videos

https://www.youtube.com/watch?v=T3LylSpYp1I

https://www.youtube.com/watch?v=Phd9yJPFQHI

https://youtu.be/7bwW9vAbN-s

