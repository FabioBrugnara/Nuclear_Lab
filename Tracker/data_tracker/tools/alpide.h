#ifndef _ALPIDE_H
#define _ALPIDE_H

#include <stdbool.h>

#define CHIPS_PER_STAVE 10
#define STAVES_N 3
#define CHIP_TOT   CHIPS_PER_STAVE * STAVES_N

#define CHIPMASK_STRING_LEN 51


enum file_meta_base {
	TYPE_DAQ       = 0x1,
	TYPE_DIGISCAN  = 0x2,
	TYPE_WHITESCAN = 0x3,
	TYPE_THRSCAN   = 0x4,
};

inline bool is_stave_brdcast(uint16_t chip_id){
	return ((chip_id&0xf) == 0xf);
};
inline bool is_gen_brdcast(uint16_t chip_id){
	return ((chip_id>>8) == 0xf);
};

int chipid_to_index(uint16_t chip_id){
	uint8_t stave_id = chip_id>>8;
	uint8_t l_chip_id = chip_id&0xf;

	int chip_index = stave_id*10;
	if (l_chip_id <= 4)
		chip_index += l_chip_id;
	else
		chip_index += l_chip_id-3;

	return chip_index;
}

uint16_t index_to_chipid(uint16_t index){
	uint8_t stave_id = index/10;

	uint8_t chip_id_lsb =  index%10;
	if (chip_id_lsb > 4)
		chip_id_lsb += 3;

	uint16_t chip_id = 0x70 | (stave_id<<8) | chip_id_lsb;
	return chip_id;
}

int printable_chipmask(uint16_t chipmask, char *chipid_list){
	int count = 0;

	for(int i=0; i<10; i++){
		if(chipmask&0x1){
			sprintf(chipid_list + count*5, "%#x ", index_to_chipid(i));
			count++;
		}
		chipmask >>= 1;
	}

	return count;
}

#endif
