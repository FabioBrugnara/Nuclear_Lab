#ifndef _DAQ_UTIL_H
#define _DAQ_UTIL_H

#include <vector>
#include <string>
#include <libftdi1/ftdi.h>

//#define ANSI_COLOR_RED     "\x1b[31m"
//#define ANSI_COLOR_GREEN   "\x1b[32m"
//#define ANSI_COLOR_YELLOW  "\x1b[33m"
//#define ANSI_COLOR_MAGENTA "\x1b[35m"
//#define ANSI_COLOR_RESET   "\x1b[0m"

#define PKT_MAX_SIZE 8000
#define THRSH_PKTS_PER_STEP 10240
#define DIGIS_PKT_NUM 5120


enum TDAQ_command {
	DAQ_PKT_PING      = 0x1,
	DAQ_RUN_CHIPSCAN  = 0x2,
	DAQ_RUN_INIT      = 0x3,
	DAQ_RUN_THRSCN    = 0x4,
	DAQ_SND_ALP_CMD   = 0x5,
	DAQ_WR_REG        = 0x6,
	DAQ_RD_REG        = 0x15,
	DAQ_MASK_PIX      = 0x7,
	DAQ_RUN_HOTPIXSCN = 0x8,
	DAQ_WR_FLASH_FILE = 0x9,
	DAQ_RD_FLASH_FILE = 0x10,
	DAQ_FORMAT_FLASH  = 0x11,
	DAQ_FALSH_LIST    = 0x12,
	DAQ_RUN_FF_TEST   = 0x13,
	DAQ_RUN_DIGISCAN  = 0x14,
	DAQ_RD_TEMP       = 0xb,
	DAQ_RD_ALP_TEMP   = 0xc
};


enum ALPIDE_OPCode {
	FSM_SET_MSK  = 0xf0,
	FSM_RD_MSK   = 0xf1,
	ALPIDE_GRST  = 0xd2,
	ALPIDE_PRST  = 0xe4,
	ALPIDE_TRIG  = 0x55,
	ALPIDE_WROP  = 0x9c,
	ALPIDE_RDOP  = 0x4e,
	ALPIDE_RORST = 0x63,
	ALPIDE_PULSE = 0x78
};


enum ALPIDE_reg_addr {
	ALPIDE_REG_MODE_CTRL  = 0x1,
	ALPIDE_REG_FROMU1     = 0x4,
	ALPIDE_REG_FROMU2     = 0x5,
	ALPIDE_REG_FROMU3     = 0x6,
	ALPIDE_REG_PULSING1   = 0x7,
	ALPIDE_REG_PULSING2   = 0x8,
	ALPIDE_REG_CMU_DMU    = 0x10,

	ALPIDE_REG_PIXCFG   = 0x500,
	ALPIDE_REG_VRESETP  = 0x601,
	ALPIDE_REG_VRESETD  = 0x602,
	ALPIDE_REG_VCASP    = 0x603,
	ALPIDE_REG_VCASN    = 0x604,
	ALPIDE_REG_VCASN2   = 0x607,
	ALPIDE_REG_VCLIP    = 0x608,
	ALPIDE_REG_VTEMP    = 0x609,
	ALPIDE_REG_IAUX2    = 0x60a,
	ALPIDE_REG_IRESET   = 0x60b,
	ALPIDE_REG_IDB      = 0x60c,
	ALPIDE_REG_IBIAS    = 0x60d,
	ALPIDE_REG_ITHR     = 0x60e
};


struct DataPacket{
	std::vector<uint8_t> buffer;
	// header fields
	uint16_t length;
	uint16_t crc;
	bool built;

	DataPacket(std::vector<uint8_t> data);
	template <typename T> int push_data(T data);
	int push_byte_vec(std::vector<uint8_t> data);
	std::vector<uint8_t> build_pkt();
	int is_status_reply();
	template <typename T> std::vector<T> get_payload();
};


class FtdiFifo{
	struct ftdi_context *ftdi;

	public:
		FtdiFifo();
		~FtdiFifo();
		std::vector<uint8_t> read_fifo();
		int write_fifo(std::vector<uint8_t> data);
};


class TDAQBoard{
	FtdiFifo comm_driver;

	bool ext_trigger_running = false;
	bool self_trigger_running = false;
	uint32_t packet_count = 0;

	public:
		TDAQBoard(){};
		~TDAQBoard(){};
		// FPGA utils
		bool ping(std::vector<uint32_t>, bool eq_check_only);
		uint32_t read_fpga_reg(uint16_t reg_addr);
		int write_fpga_reg(uint32_t reg_addr, uint32_t data);
		int fpga_reset();
		// Alpide interaction
		int run_alp_init();
		std::vector<uint32_t> chipscan();
		uint16_t send_alpide_command(uint8_t opcode, uint16_t chip_id,
				uint16_t reg_addr, uint16_t reg_val);
		int readout_loop(uint32_t meta,
				uint32_t stop_after_npkts=0,
				uint32_t self_trig_freq=0,
				std::string fname="dout.raw");
		uint16_t read_chipmask(uint8_t stave_id);
		int send_pulse(uint8_t stave_id, uint8_t pulses);
		// flash memeory operations
		int format_flash();
		std::vector<uint8_t> read_flash_file(uint8_t f_index);
		int write_flash_file(uint8_t f_index, std::vector<uint8_t> data);
		int truncate_flash_file(uint8_t f_index);
		// Calibration ans test
		int run_hotpix_scan(uint8_t stave_id, uint32_t n_triggers, uint8_t hot_thresh);
		int fifo_test(uint16_t chip_id);
		int run_threshscan(uint16_t chip_id, uint8_t nsteps);
		int run_threshscan_short(uint16_t chip_id, uint8_t nsteps);
		int run_digiscan(char mode_flag, uint16_t chip_id);
		int full_stave_test(uint8_t stave_id);

		int read_temp(uint8_t stave_id);
		int read_alp_temp(uint8_t stave_id);

		int set_tsp_switch(uint8_t stave_id, uint8_t channel, uint8_t setrst);
		int set_tsp_poweron(bool enable);


	private:
		int packet_counter(uint8_t* data, uint32_t len);
		int set_start_daq(bool start);
		int set_trigger_gen(int rate);
};

#endif
