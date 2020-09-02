#ifndef CHARACTERS_H_ /* include guard */
#define CHARACTERS_H_


extern uint8_t all_chars[256][5];

extern char character_buff[4];

void characters_init();
void add_characters(char *to_add, int num_to_add);



#endif /* CHARACTERS_H_ */
