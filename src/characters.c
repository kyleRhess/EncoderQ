#include <string.h>
#include "system.h"
#include "characters.h"

char character_buff[4];

void characters_init()
{
	memset(&character_buff, 0, 4);
}

void add_character(char to_add)
{
	static uint8_t idx = 0;

	if (idx >= sizeof(character_buff)/sizeof(character_buff[0]))
	{
		character_buff[0] = character_buff[1];
		character_buff[1] = character_buff[2];
		character_buff[2] = character_buff[3];
		character_buff[3] = to_add;
	}
	else
	{
		character_buff[idx++] = to_add;
	}


//	int degreess = (int)(deltaDegrees*10.0f);
//	char str[4] = {0,0,0,0};
//	sprintf(str, "%d", (int)deltaDegrees);
//	memcpy(&spi_txBuff[0+0], &all_chars[str[0]][0], 5);
//	memcpy(&spi_txBuff[0+5], &all_chars[str[1]][0], 5);
//	memcpy(&spi_txBuff[0+5+5], &all_chars[str[2]][0], 5);
//	memcpy(&spi_txBuff[0+5+5+5], &all_chars[str[3]][0], 5);
}
