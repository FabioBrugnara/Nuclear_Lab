#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "alpide.h"

#define MAX_EVENT_SIZE 400*10 // 400 datalong * datalong_hitmap_size
#define THRSCAN_STEPS_MAX 0x9a
#define THRSCAN_CHARGE_OFF 1

struct Hit {
	uint16_t x;
	uint16_t y;
	uint16_t chip_id;
};

struct Hit_arr {
	struct Hit h[MAX_EVENT_SIZE];
	uint32_t count;
};
struct Hit_arr Hits = {.count = 0};


struct chip_hitmap{
	uint32_t hitmap[1024][512];
	uint32_t charge_hist[THRSCAN_STEPS_MAX];
};
struct chip_hitmap chip_hitmap_arr[CHIP_TOT] = {0};

uint32_t step_hitmap[THRSCAN_STEPS_MAX][1024][512];


static struct opt_flags_s {
	int export_alice_csv;
	int export_csv;
	uint16_t chip_id_sel;
} opt_flags = {0};


static struct rawfile_info_s {
	int thresscan;
	int digiscan;
	int whitescan;
	uint8_t injs;
	uint8_t charge_steps;
} rawfile_info = {0};


static uint16_t chipmask[15] = {0};
static uint64_t eventcount = 0;
static uint32_t empty_event_count = 0;
static uint32_t busy_transition_counter = 0;
static uint32_t busy_error_counter = 0;
static uint32_t truncated_event_counter = 0;
static uint32_t corrupt_event_counter = 0;


enum ALPIDE_word_type{
	alpide_stave_header  = 0xf0,
	alpide_chip_header   = 0xa0,
	alpide_chip_trailer  = 0xb0,
	alpide_region_header = 0xc0,
	alpide_data_short    = 0x40,
	alpide_data_long     = 0x00,
	alpide_stave_trailer = 0xe0,
	alpide_truncated_ev  = 0xff,
	alpide_unknown_word
};



enum ALPIDE_word_type get_data_type(uint8_t word){
	if ( (word&0xf0) == alpide_chip_header )
		return alpide_chip_header;
	else if ( (word&0xf0) == alpide_chip_trailer )
		return alpide_chip_trailer;
	else if ( (word&0xf0) == alpide_stave_trailer )
		return alpide_stave_trailer;
	else if ( (word&0xe0) == alpide_region_header )
		return alpide_region_header;
	else if ( (word&0xc0) == alpide_data_short )
		return alpide_data_short;
	else if ( (word&0xc0) == alpide_data_long )
		return alpide_data_long;
	else if ( word == alpide_truncated_ev )
		return alpide_truncated_ev;
	else if ( (word&0xf0) == alpide_stave_header )
		return alpide_stave_header;
	else
		return alpide_unknown_word;
}

uint16_t AddressToRow(uint16_t address){
	return address/2;
}

uint16_t AddressToColumn(uint16_t address, int8_t region, uint16_t dc){
	int column=region*32+dc*2;
	int addr_mod = address%4;
	int LeftRight = (addr_mod>0 && addr_mod<=2 ? 1:0);
	column+=LeftRight;
	return column;
}



int decoder_state_check(enum ALPIDE_word_type type){

	static enum ALPIDE_word_type last_word = alpide_stave_trailer;

	if(type < 0){
		printf("ERROR decoding event: word: %#x", type);
		last_word = alpide_chip_trailer;
		return -1;
	}

	if (type == alpide_truncated_ev){
		last_word = type;
		return 0;
	}

	switch (last_word){

		case alpide_stave_header:
			if (!(type == alpide_chip_header || type == alpide_stave_trailer ||
					type == alpide_truncated_ev))
				goto err;
			break;

		case alpide_chip_header:
			if (!(type == alpide_region_header))
				goto err;
			break;

		case alpide_chip_trailer:
			if (!(type == alpide_stave_trailer || type == alpide_chip_header))
				goto err;
			break;

		case alpide_region_header:
			if (!(type == alpide_data_short || type == alpide_data_long))
				goto err;
			break;

		case alpide_data_short:
		case alpide_data_long:
			if (!(type == alpide_data_short || type == alpide_data_long ||
					type == alpide_region_header || type == alpide_chip_trailer))
				goto err;
			break;

		case alpide_stave_trailer:
			if (!(type == alpide_stave_header || type == alpide_chip_trailer))
				goto err;
			break;

		case alpide_truncated_ev:
			if(!(type==alpide_stave_trailer))
				goto err;
			break;

		default:
err:
		last_word = alpide_stave_trailer;
		corrupt_event_counter++;
		return -2;
	}

	last_word = type;

	return 0;
}



int decode(const uint16_t* _line, const int len){
	const uint8_t *line = (uint8_t*) _line;

	uint8_t current_word;
	uint32_t data_word;

	uint8_t stave_id = 0;
	uint16_t chip_id = 0;
	uint32_t region = 0, address, encoder_id, hit_map;
	uint32_t x, y;
	uint16_t rdout_flags;

	static enum ALPIDE_word_type type;

	Hits.count = 0;

	for(int n=0; n<len; ++n){
		current_word = line[n];

		struct Hit h;

		type = get_data_type(current_word);

		if(decoder_state_check(type) !=0)
			return -1;

		switch (type){
			case alpide_stave_header:
				stave_id = current_word&0xf;
				break;

			case alpide_stave_trailer:
				{
					uint32_t stave_trailer_word = (line[n]<<16) | (line[n+1]<<8) | line[n+2];
					uint16_t new_chipmask = (stave_trailer_word>>10)&0x3ff;
					//printf("newchip mask: %#x\n", current_word);

					if (new_chipmask != chipmask[stave_id]){
						uint16_t chipmask_xor = chipmask[stave_id] ^ new_chipmask;
						chipmask[stave_id] = new_chipmask;

						char masked_list[CHIPMASK_STRING_LEN];
						int num = printable_chipmask(chipmask_xor, masked_list);
						printf("****** WARNING: %d new masked chips in stave %d:\n", num, stave_id);
						printf("       %s\n", masked_list);
					}
				}

				if (len-n < 8)
					n = len;
				else
					n += 2;

				break;

			case alpide_chip_header:
				chip_id = 0x70 | (stave_id<<8) | (current_word&0xf);
				n++; // skip bunch_counter
				break;

			case alpide_region_header:
				region = current_word & 0x1f;
				break;

			case alpide_data_short:
				data_word = (current_word<<8) | line[n+1];
				address = (data_word & 0x3ff);
				encoder_id = (data_word>>10) & 0xf;
				y=AddressToRow(address);
				x=AddressToColumn(address, region, encoder_id);
				h.x=x;
				h.y=y;
				h.chip_id=chip_id;
				Hits.h[Hits.count++]=h;

				n += 1;
				break;

			case alpide_chip_trailer:
				rdout_flags = (current_word>>16) & 0xf;
				if (rdout_flags == 1)
					busy_transition_counter++;
				else if (rdout_flags != 0)
					busy_error_counter++;

				break;

			case alpide_data_long:
				data_word = (current_word<<16) | (line[n+1]<<8) | line[n+2];
				address = (data_word>>8) & 0x3ff;
				encoder_id = (data_word>>18);
				hit_map = data_word & 0x7f;

				y=AddressToRow(address);
				x=AddressToColumn(address, region, encoder_id);
				h.x=x;
				h.y=y;
				h.chip_id=chip_id;
				Hits.h[Hits.count++]=h;

				for(int i=0;i<7;i++){
					if(hit_map>>i & 0x1){
						int address_map=address+(i+1);
						y=AddressToRow(address_map);
						x=AddressToColumn(address_map, region, encoder_id);
						h.x=x;
						h.y=y;
						h.chip_id=chip_id;
						Hits.h[Hits.count++]=h;
					}
				}

				n += 2;
				break;

			case alpide_truncated_ev:
				truncated_event_counter++;
				break;

			case alpide_unknown_word:
				break;
		}

	}

	if (Hits.count == 0)
		empty_event_count++;

	return 0;
}



int update_heatmap(uint32_t charge_step){

	for(uint32_t m=0; m<Hits.count; m++){

		uint16_t chip_index = chipid_to_index(Hits.h[m].chip_id);
		if (chip_index >= CHIP_TOT) continue;

		uint16_t hit_x = Hits.h[m].x;
		uint16_t hit_y = Hits.h[m].y;

		// update hitmap
		chip_hitmap_arr[chip_index].hitmap[hit_x][hit_y]++;

		// update charge histogram
		if (rawfile_info.thresscan)
			chip_hitmap_arr[chip_index].charge_hist[charge_step]++;


		if(Hits.h[m].chip_id == opt_flags.chip_id_sel){
			if (opt_flags.export_alice_csv == 1)
				step_hitmap[charge_step][hit_x][hit_y]++;
		}

	}

	return 0;
}



int write_csv(FILE* dout){

	for(uint32_t m=0; m<Hits.count; m++){

		uint16_t chip_id = Hits.h[m].chip_id;
		uint16_t hit_x = Hits.h[m].x;
		uint16_t hit_y = Hits.h[m].y;

		fprintf(dout, "%ld,%d,%d,%d\n", eventcount, chip_id, hit_x, hit_y);
		if (dout == stdout)
			fflush(stdout);
	}


	return 0;
}



int write_alice_threshmap(){

	FILE *hit_fout;
	hit_fout = fopen("alice_hitmap.txt", "w");
	if (hit_fout == NULL){
		printf("ferror\n");
		return -1;
	}

	for(int col = 0; col<1024; col++)
		for(int row = 0; row<512; row++)
			for(uint32_t charge = 0; charge<rawfile_info.charge_steps; charge++)
				fprintf(hit_fout, "%i %i %i %i\n", col, row, charge+THRSCAN_CHARGE_OFF,
						step_hitmap[charge][col][row]);

	fclose(hit_fout);

	return 0;

}



int decode_raw(uint16_t* dataraw, uint32_t buff_len, FILE* csv_file){

	uint32_t rd_ptr = 0;

	static uint64_t charge_step=0;

	// seek and decode data packets
	while(rd_ptr < buff_len){
		if (dataraw[rd_ptr] != 0xffff) break;
		if (rd_ptr+6 > buff_len) break;

		uint16_t len = dataraw[rd_ptr+1];
		//uint16_t crc = dataraw[rd_ptr+2];
		//uint16_t trig_cnt = dataraw[rd_ptr+3];

		if (rd_ptr+6 + len/2 > buff_len) break; // ignore truncated packet at EOF
		rd_ptr += 6; // skip header

		// decode
		decode(&dataraw[rd_ptr], len);
		update_heatmap(charge_step);

		if (rawfile_info.thresscan == 1) {
			if ( (eventcount+1)%(512*rawfile_info.injs) == 0)
				charge_step++;
		}

		if (csv_file != NULL)
			write_csv(csv_file);

		// move to next event
		rd_ptr += len/2 ;
		eventcount++;
	}

	return rd_ptr;
}


int decode_file_header(uint16_t* header){

	enum file_meta_base ftype = (header[0]&0xff);

	switch (ftype) {

		case TYPE_DAQ:
			break;
		case TYPE_DIGISCAN:
			rawfile_info.digiscan = 1;
			break;
		case TYPE_WHITESCAN:
			rawfile_info.whitescan = 1;
			break;
		case TYPE_THRSCAN:
			rawfile_info.thresscan = 1;
			rawfile_info.charge_steps = (header[0]>>8)&0xff;
			rawfile_info.injs = header[1]&0xff;
			break;
		default:
			return -1;
	}

	return 0;
}


int decode_file(char* fname, int follow){

	#define buffer_size 8000
	uint16_t buffer[buffer_size];
	int retcode = 0;

	// Open raw datafile
	int data_fd = open(fname, O_RDONLY);
	if (data_fd < 0){
		printf("ferror opening %s\n", fname);
		return -1;
	}

	// Open output csv if requested
	FILE* csv_file = NULL;
	if (opt_flags.export_csv){
		csv_file = fopen("data.csv", "w");
		if (data_fd < 0){
			printf("ferror opening data.csv\n");
			return -1;
		}

		// csv header
		fprintf(csv_file, "# event_count, chip_id(decimal), hit_x, hit_y\n");
	}

	// raw file info
	struct stat data_sb;
	fstat(data_fd, &data_sb);
	off_t file_off_p = 0, buff_proc_off;

	// read file 32-bit header
	if (pread(data_fd, buffer, 4, 0) != 4){
		printf("Error reading file header\r");
		return -1;
	}
	file_off_p += 4;

	if (decode_file_header(buffer)){
		printf("Error decoding file header\r");
		return -1;
	}

	// Decode
	while(1){

		size_t rd_bytes = pread(data_fd, buffer,
				buffer_size*sizeof(uint16_t), file_off_p);
		file_off_p += rd_bytes;

		if (rd_bytes == 0){
			//printf("zero. off %d\n", file_off_p);
			sleep(1);
			continue;
		}
		//if (rd_bytes%2 != 0) printf("fuck\n");

		uint32_t n_words = rd_bytes/2;
		buff_proc_off = decode_raw(buffer, n_words, csv_file);

		// err check
		if (follow && buff_proc_off == 0){
			printf("Unexpected EoF or truncated file found\n");
			retcode = -1;
			break;
		}

		// exit on EOF if not in follow mode
		if (follow && data_sb.st_size == file_off_p)
			break;

		if (buff_proc_off != n_words){
			file_off_p -= (n_words - buff_proc_off)*2;
		}

	}

	close(data_fd);
	if (csv_file != NULL)
		fclose(csv_file);

	return retcode;
}


void set_opt(struct opt_flags_s *opts){
	opt_flags = *opts;
}

void get_rawfile_info(struct rawfile_info_s *py_rawfile_info){
	*py_rawfile_info = rawfile_info;
}

void get_hitmap(uint16_t index, uint32_t *hitmap){
	memcpy(hitmap, chip_hitmap_arr[index].hitmap, 1024*512*sizeof(uint32_t));
}

void get_histo(uint16_t index, uint32_t *histo){
	memcpy(histo, chip_hitmap_arr[index].charge_hist,
			sizeof(uint32_t)*rawfile_info.charge_steps);
}

void print_summary(){

	// print statistics
	printf("Event/trigger count: %li\n", eventcount);
	printf("Empty events: %i\n", empty_event_count);
	printf("Events with data: %li\n", eventcount-empty_event_count);
	printf("Busy transition count: %i\n", busy_transition_counter);
	printf("Busy error count: %i\n", busy_error_counter);
	printf("Truncated events: %i\n", truncated_event_counter);
	printf("Corrupt events: %i\n", corrupt_event_counter);

}
