#include "UCAM3.h"
#include "UCAM3_port.h"
#include <stdint.h>


void UCAM3_Init()
{
	ResetCam();

	HAL_Delay(1000);

	UCAM3_Sync();
	UCAM3_InitPort();
}






void ResetCam()
{
	/*
	uint8_t k = 0;  // так как k точно не будет больше 255. По идее же всё будет норм работать? Я так везде сделал
    uint8_t ResetCmd[6] = { 0xAA, 0x08, UCAM3_HardReset, 0x00, 0x00, 0x00 };
    uint8_t Buff[6];

    do
    {
    	UCAM3_Transmit(ResetCmd, 6);
		UCAM3_Receive(Buff, 6, 50);
		k++;
    }
    while (Buff[0] != UCAM3_ACKh && Buff[1] != UCAM3_ACKl && k < 20);

	if(k >= 20)
	{
		ResetCam();
	}
	*/
}

void UCAM3_InitPort()
{
    uint8_t k = 0;
    uint8_t InitCmd[6] = { 0xAA, 0x01, 0x00, UCAM3_ImageFormat_JPEG, UCAM3_RAWResolution_JPEG, UCAM3_JPEGResolution_640x480 };
    uint8_t Buff[6];

    do
    {
    	UCAM3_Transmit(InitCmd, 6);
		UCAM3_Receive(Buff, 6, 50);
		k++;
    }
    while (Buff[0] != UCAM3_ACKh && Buff[1] != UCAM3_ACKl && k < 10);

    if (k >= 10)
    {
        ResetCam();
    }
}

void UCAM3_Sync()
{
	uint8_t k = 0;
	uint8_t SyncCmd[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};
	uint8_t Buff[12];

	do
	{
		UCAM3_Transmit(SyncCmd, 6);
		UCAM3_Receive(Buff, 12, 50);
		k++;
	}
	while(Buff[0] != UCAM3_ACKh && Buff[1] != UCAM3_ACKl && k < 40);

	if(k >= 40)
	{
		ResetCam();
	}
	else
	{
		if(Buff[5] == 0xAA && Buff[6] == 0x0D) //проверка: правильно ли камера отправила запрос синхронизации?
		{
			uint8_t ACKCmd[6] = { UCAM3_ACKh, UCAM3_ACKl, 0x00, 0x00, 0x44, 0x00 };
			UCAM3_Transmit(ACKCmd, 6);
		}
	}
}

void GetPucture(int n) // n - номер фотографии
{
	uint32_t Size = 0, PackNum = 0, k = 0;
	uint8_t IsOk = 1;

	uint8_t SetPackSizeCmd[6] = {0xAA, 0x06, 0x08, 0xC8, 0x00, 0x00};
	uint8_t GetPictureCmd[6] = {0xAA, 0x04, UCAM3_PictureType_JPEG, 0x00, 0x00, 0x00};
	uint8_t Buff[6];

	do
	{
		UCAM3_Transmit(SetPackSizeCmd, 6);
		UCAM3_Receive(Buff, 6, 10);
		k++;
	}
	while(Buff[0] != UCAM3_ACKh && Buff[1] != UCAM3_ACKl && k < 20);

	if(k >= 20) {IsOk = 0;}
	else
	{
		for(uint8_t i = 0; i < 6; i++) //сбрасываем буфер
		{
			Buff[i] = 0x00;
		}
		k = 0;

		do
		{
			UCAM3_Transmit(GetPictureCmd, 6);
			UCAM3_Receive(Buff, 6, 5);
			k++;
		}
		while(Buff[0] != UCAM3_ACKh && Buff[1] != UCAM3_ACKl && k < 20);

		if(k >= 20)	{IsOk = 0;}
		else
		{
			for(uint8_t i = 0; i < 6; i++) //сбрасываем буфер
			{
				Buff[i] = 0x00;
			}
			k = 0;

			do
			{
				UCAM3_Receive(Buff, 6, 5);
				k++;
			}
			while(Buff[0] != 0xAA && Buff[1] != 0x0A && Buff[2] != 0x01 && k < 20);

			if (k >= 20) {IsOk = 0;}
			else
			{
				Size = (Buff[3] << 0) | (Buff[4] << 8) | (Buff[5] << 16);
				PackNum = Size / 194;
				if (Size % 194 != 0) {PackNum++;}

				uint16_t i = 0;
				while (i < PackNum && IsOk == 1)
				{
					uint8_t First = (uint8_t)(i << 8);
					uint8_t Last = (uint8_t)(i & 0xFF);
					uint8_t PictureCmd[6] = { UCAM3_ACKh, UCAM3_ACKl, 0x00, 0x00, Last, First };
					k = 0;

					if (i != PackNum)
					{
						uint8_t ImageBuff[200];
						do
						{
							UCAM3_Transmit(PictureCmd, 6);
							UCAM3_Receive(ImageBuff, 200, 5);
							k++;
						}
						while(ImageBuff[199] == 0x00 && k < 20); //вот тут проверку лучше переделать

						if (k >= 100) {IsOk = 0;}
						else
						{
							WriteOnSD(n, &ImageBuff[4], 194);
							/*
							 * Запись информации, отделяем первые 4 байта и не доходим до последних 2
							for (int j = 4; j < 198; j++)
							{
								 = ImageBuff[j];
							}
							*/
						}
					}

					else
					{
						uint8_t PackSize = Size - PackNum*200 + 6; // +6 это +6 байт информации, т.к.
						uint8_t ImageBuff[PackSize];			   // в Size только размер картинки
						do
						{
							UCAM3_Transmit(PictureCmd, 6);
							UCAM3_Receive(ImageBuff, PackSize, 50);
							k++;
						}
						while(ImageBuff[PackSize - 1] == 0x00 && k < 20); //вот тут проверку лучше переделать

						if (k >= 10){IsOk = 0;}
						else
						{
							WriteOnSD(n, &ImageBuff[4], PackSize - 6);

							/*
							 * Запись информации, отделяем первые 4 байта и не доходим до последних 2
							for (int j = 4; j < PackSize - 2; j++)
							{
								 = ImageBuff[j];
							}
							*/
						}
					}

					i++;
				} // конец while

            	if (IsOk == 1)
            	{
            		uint8_t ACKEndCmd[6] = {UCAM3_ACKh, UCAM3_ACKl, 0x00, 0x00, 0xF0, 0xF0};
        			UCAM3_Transmit(ACKEndCmd, 6);
            	}
			}
		}
	}
}
