#include <string.h>
#include "system.h"
#include "characters.h"

char character_buff[4];

void characters_init()
{
	memset(&character_buff, 0, 4);
}

void add_characters(char *to_add, int num_to_add)
{
	static uint8_t idx = 0;

	for (int i = 0; i < num_to_add; ++i)
	{
		if (idx >= sizeof(character_buff)/sizeof(character_buff[0]))
		{
			character_buff[0] = character_buff[1];
			character_buff[1] = character_buff[2];
			character_buff[2] = character_buff[3];
			character_buff[3] = to_add[i];
		}
		else
		{
			character_buff[idx++] = to_add[i];
		}
	}


//	int degreess = (int)(deltaDegrees*10.0f);
//	char str[4] = {0,0,0,0};
//	sprintf(str, "%d", (int)deltaDegrees);
//	memcpy(&spi_txBuff[0+0], &all_chars[str[0]][0], 5);
//	memcpy(&spi_txBuff[0+5], &all_chars[str[1]][0], 5);
//	memcpy(&spi_txBuff[0+5+5], &all_chars[str[2]][0], 5);
//	memcpy(&spi_txBuff[0+5+5+5], &all_chars[str[3]][0], 5);
}

uint8_t all_chars[256][5] = {
//		0 - 31
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00},

//		32 - 127
	{0x00, 0x00, 0x00, 0x00, 0x00}, // Space
	{0x00, 0x00, 0x5F, 0x00, 0x00}, // !
	{0x00, 0x03, 0x00, 0x03, 0x00}, // "
	{0x14, 0x3E, 0x14, 0x3E, 0x14}, // #
	{0x24, 0x2A, 0x6B, 0x2A, 0x12}, // $
	{0x26, 0x12, 0x08, 0x24, 0x32}, // %
	{0x76, 0x49, 0x56, 0x20, 0x50}, // &
	{0x00, 0x00, 0x03, 0x00, 0x00}, // '
	{0x1C, 0x22, 0x41, 0x00, 0x00}, // (
	{0x00, 0x00, 0x41, 0x22, 0x1C}, // )
	{0x00, 0x05, 0x02, 0x05, 0x00}, // *
	{0x08, 0x08, 0x3E, 0x08, 0x08}, // +
	{0x00, 0x10, 0x60, 0x00, 0x00}, // ,
	{0x08, 0x08, 0x08, 0x08, 0x08}, // -
	{0x00, 0x60, 0x60, 0x00, 0x00}, // .
	{0x00, 0x60, 0x18, 0x06, 0x01}, // /
	{0x3E, 0x41, 0x41, 0x41, 0x3E}, // 0
	{0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
	{0x42, 0x61, 0x51, 0x49, 0x46}, // 2
	{0x22, 0x41, 0x49, 0x49, 0x36}, // 3
	{0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
	{0x4F, 0x49, 0x49, 0x49, 0x31}, // 5
	{0x3E, 0x49, 0x49, 0x49, 0x32}, // 6
	{0x61, 0x31, 0x19, 0x0D, 0x07}, // 7
	{0x36, 0x49, 0x49, 0x49, 0x36}, // 8
	{0x26, 0x49, 0x49, 0x49, 0x3E}, // 9
	{0x00, 0x00, 0x22, 0x00, 0x00}, // :
	{0x00, 0x40, 0x32, 0x00, 0x00}, // ;
	{0x00, 0x00, 0x08, 0x14, 0x22}, // <
	{0x00, 0x14, 0x14, 0x14, 0x00}, // =
	{0x22, 0x14, 0x08, 0x00, 0x00}, // >
	{0x02, 0x01, 0x51, 0x09, 0x06}, // ?
	{0x3E, 0x49, 0x55, 0x5D, 0x0E}, // @
	{0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
	{0x7F, 0x49, 0x49, 0x4E, 0x30}, // B
	{0x3E, 0x41, 0x41, 0x41, 0x22}, // C
	{0x7F, 0x41, 0x41, 0x3E, 0x00}, // D
	{0x7F, 0x49, 0x49, 0x41, 0x00}, // E
	{0x7F, 0x09, 0x09, 0x01, 0x00}, // F
	{0x3E, 0x41, 0x51, 0x51, 0x32}, // G
	{0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
	{0x00, 0x41, 0x7F, 0x41, 0x00}, // I
	{0x20, 0x41, 0x3F, 0x01, 0x00}, // J
	{0x7F, 0x08, 0x14, 0x22, 0x41}, // K
	{0x7F, 0x40, 0x40, 0x40, 0x00}, // L
	{0x7E, 0x04, 0x08, 0x04, 0x7E}, // M
	{0x7C, 0x08, 0x10, 0x20, 0x7C}, // N
	{0x3C, 0x42, 0x42, 0x42, 0x3C}, // O
	{0x7C, 0x12, 0x12, 0x12, 0x0C}, // P
	{0x1E, 0x21, 0x21, 0x21, 0x5E}, // Q
	{0x7E, 0x09, 0x19, 0x29, 0x46}, // R
	{0x26, 0x49, 0x49, 0x49, 0x32}, // S
	{0x02, 0x02, 0x7E, 0x02, 0x02}, // T
	{0x3E, 0x40, 0x40, 0x40, 0x3E}, // U
	{0x1C, 0x20, 0x40, 0x20, 0x1C}, // V
	{0x3E, 0x40, 0x30, 0x40, 0x3E}, // W
	{0x42, 0x24, 0x18, 0x24, 0x42}, // X
	{0x06, 0x18, 0x70, 0x18, 0x06}, // Y
	{0x42, 0x62, 0x52, 0x4A, 0x46}, // Z
	{0x00, 0x7F, 0x41, 0x00, 0x00}, // [
	{0x01, 0x06, 0x08, 0x30, 0x40}, // \*
	{0x00, 0x00, 0x41, 0x7F, 0x00}, // ]
	{0x04, 0x02, 0x01, 0x02, 0x04}, // ^
	{0x40, 0x40, 0x40, 0x40, 0x40}, // _
	{0x00, 0x01, 0x02, 0x00, 0x00}, // `
	{0x20, 0x54, 0x54, 0x38, 0x00}, // a
	{0x7C, 0x50, 0x50, 0x20, 0x00}, // b
	{0x30, 0x48, 0x48, 0x00, 0x00}, // c
	{0x00, 0x20, 0x50, 0x50, 0x7C}, // d
	{0x38, 0x54, 0x54, 0x18, 0x00}, // e
	{0x10, 0x78, 0x14, 0x04, 0x00}, // f
	{0x00, 0x24, 0x4A, 0x4A, 0x3C}, // g
	{0x00, 0x7C, 0x10, 0x60, 0x00}, // h
	{0x00, 0x10, 0x74, 0x00, 0x00}, // i
	{0x20, 0x40, 0x3A, 0x00, 0x00}, // j
	{0x00, 0x7C, 0x10, 0x68, 0x00}, // k
	{0x00, 0x04, 0x7C, 0x00, 0x00}, // l
	{0x70, 0x10, 0x60, 0x10, 0x60}, // m
	{0x00, 0x70, 0x10, 0x60, 0x00}, // n
	{0x00, 0x70, 0x50, 0x60, 0x00}, // o
	{0x00, 0x7C, 0x14, 0x18, 0x00}, // p
	{0x00, 0x18, 0x14, 0x7C, 0x00}, // q
	{0x00, 0x78, 0x08, 0x10, 0x00}, // r
	{0x00, 0x48, 0x54, 0x24, 0x00}, // s
	{0x00, 0x08, 0x3C, 0x48, 0x00}, // t
	{0x00, 0x30, 0x40, 0x70, 0x00}, // u
	{0x00, 0x30, 0x40, 0x30, 0x00}, // v
	{0x30, 0x40, 0x20, 0x40, 0x30}, // w
	{0x44, 0x28, 0x10, 0x28, 0x44}, // x
	{0x00, 0x4C, 0x50, 0x3C, 0x00}, // y
	{0x00, 0x48, 0x68, 0x58, 0x00}, // z
	{0x00, 0x08, 0x77, 0x41, 0x00}, // {
	{0x00, 0x00, 0x7F, 0x00, 0x00}, // |
	{0x00, 0x41, 0x77, 0x08, 0x00}, // }
	{0x04, 0x02, 0x04, 0x08, 0x04}, // ~
	{0x00, 0x00, 0x00, 0x00, 0x00}, // delete

//		128 - 255
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}, // unknown
	{0x7E, 0x42, 0x42, 0x42, 0x7E}
};
