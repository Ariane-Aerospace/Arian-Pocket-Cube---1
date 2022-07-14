#include "Transmitter.h"


uint32_t FloatToBin(float A)
{

	float* pA = &A;
	uint32_t* pB = (uint32_t*)pA;
	uint32_t B = *pB;
	return B;
}

void SetRadioData(Telemetry* Telemetry, uint8_t* data)
{

	data[0] = '$';
	data[1] = 'P';

	data[2] = (uint8_t)(Telemetry->Ticks>>24);
	data[3] = (uint8_t)(Telemetry->Ticks>>16);
	data[4] = (uint8_t)(Telemetry->Ticks>>8);
	data[5] = (uint8_t)(Telemetry->Ticks);

	uint16_t IntTemp = FloatToBin(Telemetry->DS18_temp);
	data[6] = (uint8_t)(IntTemp>>8);
	data[7] = (uint8_t)(IntTemp);

	data[8] = (uint8_t)(Telemetry->MS5607_pres>>16);
	data[9] = (uint8_t)(Telemetry->MS5607_pres>>8);
	data[10] = (uint8_t)(Telemetry->MS5607_pres);

	//uint32_t IntHeight = FloatToBin(Telemetry->Height);
	uint32_t IntHeight = 123435;
	data[11] = (uint8_t)(IntHeight>>24);
	data[12] = (uint8_t)(IntHeight>>16);
	data[13] = (uint8_t)(IntHeight>>8);
	data[14] = (uint8_t)(IntHeight);

	//uint32_t IntLongitude = FloatToBin(Telemetry->Longtitude);
	uint32_t IntLongitude = 2334213;
	data[15] = (uint8_t)(IntLongitude>>24);
	data[16] = (uint8_t)(IntLongitude>>16);
	data[17] = (uint8_t)(IntLongitude>>8);
	data[18] = (uint8_t)(IntLongitude);

	//uint32_t IntAttitude = FloatToBin(Telemetry->Alttitude);
	uint32_t IntAttitude = 355435;
	data[19] = (uint8_t)(IntAttitude>>24);
	data[20] = (uint8_t)(IntAttitude>>16);
	data[21] = (uint8_t)(IntAttitude>>8);
	data[22] = (uint8_t)(IntAttitude);
	data[22] = 0x0A;

}

void SetAllData(Telemetry* Telemetry, uint8_t* data)
{
	data[0] = 0x0D;
	data[1] = '$';

	data[2] = (uint8_t)(Telemetry->Ticks>>24);
	data[3] = (uint8_t)(Telemetry->Ticks>>16);
	data[4] = (uint8_t)(Telemetry->Ticks>>8);
	data[5] = (uint8_t)(Telemetry->Ticks);

	data[6] = (uint8_t)(Telemetry->Vbat>>8);
	data[7] = (uint8_t)(Telemetry->Vbat);

	data[8] = (uint8_t)(Telemetry->I_DS>>8);
	data[9] = (uint8_t)(Telemetry->I_DS);

	data[10] = (uint8_t)(Telemetry->I_CH>>8);
	data[11] = (uint8_t)(Telemetry->I_CH);

	data[12] = (uint8_t)(Telemetry->DS18_temp>>8);
	data[13] = (uint8_t)(Telemetry->DS18_temp);

	data[14] = (uint8_t)(Telemetry->AccX>>8);
	data[15] = (uint8_t)(Telemetry->AccX);

	data[16] = (uint8_t)(Telemetry->AccY>>8);
	data[17] = (uint8_t)(Telemetry->AccY);

	data[18] = (uint8_t)(Telemetry->AccZ>>8);
	data[19] = (uint8_t)(Telemetry->AccZ);

	data[20] = (uint8_t)(Telemetry->GyroX>>8);
	data[21] = (uint8_t)(Telemetry->GyroX);

	data[22] = (uint8_t)(Telemetry->GyroY>>8);
	data[23] = (uint8_t)(Telemetry->GyroY);

	data[24] = (uint8_t)(Telemetry->GyroZ>>8);
	data[25] = (uint8_t)(Telemetry->GyroZ);

	data[26] = (uint8_t)(Telemetry->MagX>>8);
	data[27] = (uint8_t)(Telemetry->MagX);

	data[28] = (uint8_t)(Telemetry->MagY>>8);
	data[29] = (uint8_t)(Telemetry->MagY);

	data[30] = (uint8_t)(Telemetry->MagZ>>8);
	data[31] = (uint8_t)(Telemetry->MagZ);

	data[32] = (uint8_t)(Telemetry->MS5607_pres>>24);
	data[33] = (uint8_t)(Telemetry->MS5607_pres>>16);
	data[34] = (uint8_t)(Telemetry->MS5607_pres>>8);
	data[35] = (uint8_t)(Telemetry->MS5607_pres);

	//uint32_t IntHeight = FloatToBin(Telemetry->Height);
	uint32_t IntHeight = 123435;
	data[36] = (uint8_t)(IntHeight>>24);
	data[37] = (uint8_t)(IntHeight>>16);
	data[38] = (uint8_t)(IntHeight>>8);
	data[39] = (uint8_t)(IntHeight);

	//uint32_t IntLongitude = FloatToBin(Telemetry->Longtitude);
	uint32_t IntLongitude = 2334213;
	data[40] = (uint8_t)(IntLongitude>>24);
	data[41] = (uint8_t)(IntLongitude>>16);
	data[42] = (uint8_t)(IntLongitude>>8);
	data[43] = (uint8_t)(IntLongitude);

	//uint32_t IntAttitude = FloatToBin(Telemetry->Alttitude);
	uint32_t IntAttitude = 355435;
	data[44] = (uint8_t)(IntAttitude>>24);
	data[45] = (uint8_t)(IntAttitude>>16);
	data[46] = (uint8_t)(IntAttitude>>8);
	data[47] = (uint8_t)(IntAttitude);

}

