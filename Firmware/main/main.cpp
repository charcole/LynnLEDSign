// (c) Charlie Cole 2018

#include <stdio.h>
#include <math.h>
#include <string.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
};

#define OUT_CLOCK  (GPIO_NUM_19)
#define OUT_LATCH  (GPIO_NUM_18)
#define OUT_ENABLE (GPIO_NUM_17)
#define OUT_DATA   (GPIO_NUM_16)

bool PWMState[48] = { false };
uint16_t PWMCounters[48] = { 0 };
uint16_t PWMSteps[48] = { 0 }; // 256 is fully on, 0 is off

void UpdatePWM()
{
	for (int32_t i=0; i<48; i++)
	{
		PWMState[i] = (PWMCounters[i] + PWMSteps[i] >= 0x100);
		PWMCounters[i] = (PWMCounters[i] + PWMSteps[i]) & 0xFF;
	}
}

void ShiftOut32(uint32_t Shift)
{
	for (int i=0; i<32; i++)
	{
		gpio_set_level(OUT_CLOCK, 0);
		ets_delay_us(1);
		gpio_set_level(OUT_DATA, ((Shift&0x80000000)==0)?1:0);
		ets_delay_us(1);
		gpio_set_level(OUT_CLOCK, 1);
		ets_delay_us(1);
		Shift<<=1;
	}
}

void ShiftOut16(uint32_t Shift)
{
	for (int i=0; i<16; i++)
	{
		gpio_set_level(OUT_CLOCK, 0);
		ets_delay_us(1);
		gpio_set_level(OUT_DATA, ((Shift&0x8000)==0)?1:0);
		ets_delay_us(1);
		gpio_set_level(OUT_CLOCK, 1);
		ets_delay_us(1);
		Shift<<=1;
	}
}

void PWMTask(void *pvParameters)
{
	printf("PWMTask running on core %d\n", xPortGetCoreID());

	while (true)
	{
		uint32_t ShiftA = 0;
		uint32_t ShiftB = 0;
		UpdatePWM();
		for (int32_t i=0; i<32; i++)
		{
			ShiftA <<= 1;
			ShiftA |= PWMState[i]?0:1;
		}
		for (int32_t i=0; i<16; i++)
		{
			ShiftB <<= 1;
			ShiftB |= PWMState[32+i]?0:1;
		}

		ShiftOut32(ShiftA);
		ShiftOut16(ShiftB);

		// Latch it
		gpio_set_level(OUT_LATCH, 1);
		ets_delay_us(1);
		gpio_set_level(OUT_LATCH, 0);
		ets_delay_us(1);
		gpio_set_level(OUT_ENABLE, 0);
	}
}

float LEDPositions[45][2]=
{
	{241.25f, 94.25f},
	{234.6f, 78.5f},
	{229.4f, 81.3f},
	{234.7f, 84.3f},
	{231.3f, 89.3f},
	{229.4f, 95.4f},
	{220.2f, 89.1f},
	{217.3f, 94.4f},
	{206.5f, 100.9f},
	{214.8f, 100.1f},
	{231.2f, 101.1f},
	{237.9f, 98.8f},
	{223.1f, 84.6f},
	{225.3f, 78.7f},
	{205.1f, 95.9f},
	{207.4f, 89.9f},
	{197.5f, 85.0f},
	{201.0f, 78.7f},
	{205.2f, 81.4f},
	{210.6f, 78.8f},
	{210.3f, 84.8f},
	{195.7f, 90.2f},
	{192.1f, 96.0f},
	{185.1f, 100.6f},
	{178.6f, 104.0f},
	{170.2f, 107.1f},
	{164.9f, 113.2f},
	{165.45f, 122.5f},
	{171.3f, 117.5f},
	{175.4f, 110.5f},
	{180.7f, 97.1f},
	{183.7f, 91.0f},
	{186.5f, 85.3f},
	{189.4f, 79.4f},
	{172.2f, 100.5f},
	{169.2f, 94.1f},
	{172.7f, 87.8f},
	{176.5f, 80.8f},
	{160.6f, 98.3f},
	{150.8f, 100.0f},
	{148.9f, 93.5f},
	{152.2f, 86.75f},
	{155.5f, 80.9f},
	{159.7f, 74.0f},
	{162.9f, 68.1f}
};

int8_t Write[]=
{
	44,43,42,41,40,39,38,-1,-1,-1,-1, // L
	37,36,35,34,30,31,32,33,33,33, // Top of y
	32,31,30,24,29,28,27,26,25,24,23,22,21,16,17,17, // y to n
	16,21,22,22, // n down
	21,16,18,19,20,15,14,8,9,7,6,12,13,13, // n to n
	12,6,7,9,9, // n down
	7,6,12,2,1,3,4,5,10,11,0 // finish n
};

int8_t Letters[4][15]=
{
	{ 3, 4, 5, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,-1,-1},
	{10,11,12,13,14,15,16,17,18,19,20,21,22,23,24},
	{25,26,27,28,29,30,31,32,33,39,-1,-1,-1,-1,-1},
	{34,35,36,37,38,40,41,42,43,44,45,46,47,-1,-1}
};

enum EState
{
	State_Write,
	State_Sweep,
	State_Letters,
	State_Emit,
	State_Max
};

int StateList[]=
{
	State_Write,
	State_Emit,
	State_Sweep,
	State_Emit,
	State_Letters,
	State_Sweep,
	State_Emit,
	State_Sweep
};

int StateRandom[]=
{
	State_Letters,
	State_Emit,
	State_Sweep,
	State_Emit,
	State_Sweep,
	State_Letters,
	State_Sweep,
	State_Emit,
	State_Write
};

#define ARRAY_SIZEOF(x) (sizeof(x)/sizeof((x)[0]))

void DrawingTask(void *pvParameters)
{
	printf("DrawingTask running on core %d\n", xPortGetCoreID());
	
	vTaskDelay(1000);

	int State = StateList[0];
	int StateIndex = 1;
	int LastRandomIndex = -1;
					
	while (true)
	{
		switch (State)
		{
			case State_Write:
			{
				int WriteCount = sizeof(Write) / sizeof(Write[0]);
				for (int Phase = -16; Phase < WriteCount + 2; Phase++)
				{
					int Brightness = 0;
					for (int i = 0; i < 16; i++)
					{
						if (Phase + i < WriteCount && Phase + i >= 0)
						{
							int LED = 47 - Write[Phase + i];
							if (LED >= 0 && LED < 48)
							{
								int Value = (Brightness * Brightness) >> 8;
								Value = (Value * Value) >> 8;
								PWMSteps[LED] = 4 + Value;
							}
						}
						Brightness += 15;
					}
					vTaskDelay(40);
				}
				break;
			}
			case State_Letters:
			{
				int LetterCount = sizeof(Letters[0]) / sizeof(Letters[0][0]);
				for (int Letter = 0; Letter < 4; Letter++)
				{
					for (int i=0; i<LetterCount; i++)
					{
						int LED = Letters[Letter][i];
						if (LED >= 0)
							PWMSteps[LED] = 256;
					}
					vTaskDelay(500);
					for (int i=0; i<LetterCount; i++)
					{
						int LED = Letters[Letter][i];
						if (LED >= 0)
							PWMSteps[LED] = 4;
					}
				}
				break;
			}
			case State_Sweep:
			{
				int LEDCount = sizeof(LEDPositions) / sizeof(LEDPositions[0]);
				
				float Direction[2];
				Direction[0] = 2.0f*(rand()/(float)(RAND_MAX-1))-1.0f;
				Direction[1] = 2.0f*(rand()/(float)(RAND_MAX-1))-1.0f;
				float Length = sqrtf(Direction[0]*Direction[0]+Direction[1]*Direction[1]);
				if (Length < 0.01f)
					break;
				Length = 1.0f / Length;
				Direction[0] *= Length;
				Direction[1] *= Length;

				float Min = 10000.0f;
				float Max = -10000.0f;
				float Falloff = 20.0f;
				for (int i=0; i<LEDCount; i++)
				{
					float Dot = Direction[0] * LEDPositions[i][0] + Direction[1] * LEDPositions[i][1];
					if (Dot<Min)
						Min=Dot;
					if (Dot>Max)
						Max=Dot;
				}
				Min -= Falloff;
				Max += Falloff;
				float Current = Min;
				while (Current < Max)
				{
					for (int i=0; i<LEDCount; i++)
					{
						float Dot = Direction[0] * LEDPositions[i][0] + Direction[1] * LEDPositions[i][1];
						float Dist = fabsf(Dot - Current) / Falloff;
						Dist = sqrt(Dist);
						int Fade = int(256 * (1.0f - Dist));
						if (Fade < 4)
							Fade = 4;
						PWMSteps[47-i] = Fade;
					}
					Current += 0.1f;
					vTaskDelay(2);
				}
				break;
			}
			case State_Emit:
			{
				int LEDCount = sizeof(LEDPositions) / sizeof(LEDPositions[0]);

				int CentreLED = rand() % LEDCount;
				float Centre[2] = { LEDPositions[CentreLED][0], LEDPositions[CentreLED][1] };

				float Max = 0.0f;
				float Falloff = 20.0f;
				for (int i=0; i<LEDCount; i++)
				{
					float Diff[2] =
					{
						LEDPositions[i][0] - Centre[0],
						LEDPositions[i][1] - Centre[1]
					};
					float Dist = Diff[0]*Diff[0] + Diff[1]*Diff[1];
					if (Dist>Max)
						Max=Dist;
				}
				Max = sqrt(Max);
				Max += Falloff;

				float Current = -Falloff;
				while (Current < Max)
				{
					for (int i=0; i<LEDCount; i++)
					{
						float Diff[2] =
						{
							LEDPositions[i][0] - Centre[0],
							LEDPositions[i][1] - Centre[1]
						};
						float Dist = fabsf((Current - sqrt(Diff[0]*Diff[0] + Diff[1]*Diff[1])) / Falloff);
						
						int Fade = int(256 * (1.0f - Dist));
						if (Fade < 4)
							Fade = 4;
						PWMSteps[47-i] = Fade;
					}
					Current += 0.1f;
					vTaskDelay(2);
				}
				break;
			}
		}

		if (StateIndex < ARRAY_SIZEOF(StateList))
		{
			State = StateList[StateIndex];
			StateIndex++;
		}
		else
		{
			int RandomIndex;
			do
			{
				RandomIndex = rand() % ARRAY_SIZEOF(StateRandom);
			} while (RandomIndex == LastRandomIndex);
			LastRandomIndex = RandomIndex;
			State = StateRandom[RandomIndex];
		}
	}
}


void InitializeGPIO()
{
	gpio_config_t GPIOConfig;
	GPIOConfig.intr_type = GPIO_INTR_DISABLE;
	GPIOConfig.pin_bit_mask = BIT(OUT_ENABLE);
	GPIOConfig.mode = GPIO_MODE_OUTPUT;
	GPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
	GPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&GPIOConfig);
	gpio_set_level(OUT_ENABLE, 1);

	GPIOConfig.pin_bit_mask = BIT(OUT_LATCH);
	gpio_config(&GPIOConfig);
	
	GPIOConfig.pin_bit_mask = BIT(OUT_DATA);
	gpio_config(&GPIOConfig);
	
	GPIOConfig.pin_bit_mask = BIT(OUT_CLOCK);
	gpio_config(&GPIOConfig);
}

extern "C" void app_main(void)
{
	InitializeGPIO();

	// Magic non-sense to make second core work
	vTaskDelay(500 / portTICK_PERIOD_MS);
	nvs_flash_init();

	memset(PWMState, 0, sizeof(PWMState));
	memset(PWMCounters, 0, sizeof(PWMCounters));
	memset(PWMSteps, 0, sizeof(PWMSteps));

	xTaskCreatePinnedToCore(&DrawingTask, "DrawingTask", 8192, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(&PWMTask, "PWMTask", 8192, NULL, 5, NULL, 1);
}
