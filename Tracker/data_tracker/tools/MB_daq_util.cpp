#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstring>
#include <signal.h>
#include <stdexcept>
#include <unistd.h>
#include <map>
#include "ini.h"

#include "MB_daq_util.hpp"
#include "alpide.h"



// ******** signal handler *********
static bool terminate = false;
static bool force_terminate = false;

void sigint_handler(int) {

	if (!terminate){
		std::cout << "Stopping on user request... Repeat to force terminate" << std::endl;
		terminate = true;
	}
	else if (!force_terminate) {
		std::cout << "terminate programm... Repeat again to force exit" << std::endl;
		force_terminate = true;
	}
	else{
		throw std::runtime_error("Forced exit");
	}

}
// ****************************



// ******************* DataPacket Class ********************
DataPacket::DataPacket(std::vector<uint8_t> data = {}){

	if (data.empty()){
		buffer.insert(buffer.begin(), {0x00, 0x00, 0xfe, 0xff}); // empty header
		length = 0;
		built = false;
	}
	else {
		buffer = data;

		if (buffer[1] != 0xff && buffer[0] != 0xfe){
			buffer.clear();
			std::cout << "DataPacket can only process command pacakges" << std::endl;
			return;
		}

		length = (buffer[3]<<8) | buffer[2];
		built = true;
	}

}


int DataPacket::push_byte_vec(std::vector<uint8_t> data){
	if (built)
		throw  std::runtime_error("Tried to push data to an already built package");

	if (buffer.size() > PKT_MAX_SIZE)
		throw  std::out_of_range("Tried to push data to an already built package");

	buffer.insert(buffer.end(), data.begin(), data.end());

	return 0;
}


template <typename T>
int DataPacket::push_data(T data) {
	uint8_t* src = static_cast<uint8_t*>(static_cast<void*>(&data));
	buffer.insert(buffer.end(), src, src+sizeof(T));

	return 0;
}


std::vector<uint8_t> DataPacket::build_pkt(){
	if (built)
		return buffer;

	// total size - header (4 bytes.. TODO: for now)
	uint16_t payload_size = buffer.size()-4;

	// zero padding
	int remainder = payload_size%4;
	if (remainder){
		int padding_len = 4 - remainder;
		buffer.resize(buffer.size() + padding_len, 0x00);
	}

	// set length header field
	buffer[1] = (payload_size<<8);
	buffer[0] = (payload_size&0xffff);

	built = true;

	return buffer;
}


int DataPacket::is_status_reply(){
	std::vector<uint32_t> resp = get_payload<uint32_t>();

	if (resp.size() > 1){
		std::cerr << "error: pacakge is not a status reply" << std::endl;
		return -1; // not a status reply
	}

	if (resp[0] != 0)
		return 1; // resp error

	return 0; // resp ok
}


template <typename T>
std::vector<T> DataPacket::get_payload(){
	std::vector<T> payload_cpy(length/sizeof(T));
	std::memcpy(payload_cpy.data(), buffer.data()+6, length);
	return payload_cpy;
}
// ****************************************************




// ******************* FTDI fifo Class ********************
FtdiFifo::FtdiFifo(){
	const int vendor_id = 0x403;
	const int product_id = 0x6010;
	int ret;

	ftdi = ftdi_new();
	ret = ftdi_set_interface(ftdi, INTERFACE_A);
	if (ret < 0)
		throw std::runtime_error("ftdi set_interface failed");

	//ret = ftdi_usb_open_desc_index(ftdi, vendor_id, product_id, NULL , iserial, 0);
	ret = ftdi_usb_open(ftdi, vendor_id, product_id);
	if (ret < 0){
		std::cout << "ftdi error " << ret << std::endl;
		throw std::runtime_error("unable to open device\n");
	}

	ret = ftdi_usb_reset(ftdi);
	ret |= ftdi_set_bitmode(ftdi, 0xFF, BITMODE_RESET);
	if (ret < 0)
		throw std::runtime_error("Device reset failed\n");

	ret = ftdi_set_bitmode(ftdi, 0xFF, BITMODE_SYNCFF);
	if (ret < 0)
		throw std::runtime_error("Unable to set bitmode SYNCFF\n");

	ftdi_usb_purge_buffers(ftdi);
}


FtdiFifo::~FtdiFifo(){

	if (ftdi_usb_reset(ftdi) < 0)
		std::cerr << "FTDI device reset failed" << std::endl;

	if (ftdi_set_bitmode(ftdi, 0xFF, BITMODE_RESET) < 0)
		std::cerr << " FTDI BITMODE reset failed" << std::endl;

	ftdi_usb_close(ftdi);
	ftdi_free(ftdi);

}


int FtdiFifo::write_fifo(std::vector<uint8_t> data){

	int ret = ftdi_write_data(ftdi, &data[0], data.size());

	if (ret != data.size()){
		std::cerr << "Error writing to FTDI fifo, errcode: " << ret << std::endl;
		ret = -1;
	}

	return ret;
}


std::vector<uint8_t> FtdiFifo::read_fifo(){

	if (terminate){
		ftdi_usb_purge_rx_buffer(ftdi);
		return {};
	}

	int length = 0;
	std::vector<uint8_t> data_buffer(PKT_MAX_SIZE);

	while(length == 0 && !terminate){
		length = ftdi_read_data(ftdi, &data_buffer[0], data_buffer.size());

		if (length < 0){
			std::cerr << "Error reading from FTDI fifo, errcode: " << length << std::endl;
			return {};
		}

	}

	data_buffer.resize(length);
	return data_buffer;
}
// ****************************************************




// ******************* TDAQBoard Class ********************


bool TDAQBoard::ping(std::vector<uint32_t> data, bool eq_check_only=false){

	DataPacket packet;

	packet.push_data(uint8_t(DAQ_PKT_PING));
	for (auto word: data)
		packet.push_data(word);

	comm_driver.write_fifo(packet.build_pkt());

	// read data
	DataPacket rcv_packet(comm_driver.read_fifo());

	std::vector<uint32_t> payload32 = rcv_packet.get_payload<uint32_t>();

	if (! eq_check_only)
		for(auto word: payload32)
			std::cout << std::hex << "0x" << int(word) << std::endl;

	return std::equal(data.begin(), data.end(), payload32.begin());
}


int TDAQBoard::run_alp_init(){

	DataPacket packet;
	std::vector<uint8_t> data = read_flash_file(0);
	if (data[0] == 0xff && data[1] == 0xff){
		std::cout << "Error reading alp conf from flash." << std::endl;
		return -1;
	}

	// run init
	std::cout << "run init.." << std::endl;
	packet.push_data(uint8_t(DAQ_RUN_INIT));
	comm_driver.write_fifo(packet.build_pkt());

	// read 'done' msg
	DataPacket rcv_packet(comm_driver.read_fifo());
	if (rcv_packet.is_status_reply() == 0)
		std::cout << "Done" << std::endl;
	else
		std::cout << "Error running init" << std::endl;

	return 0;
}


std::vector<uint32_t> TDAQBoard::chipscan(){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RUN_CHIPSCAN));
	comm_driver.write_fifo(packet.build_pkt());

	std::vector<uint8_t> rcv_data = comm_driver.read_fifo();
	DataPacket resp_pkt(rcv_data);

	std::vector<uint32_t> payload32 = resp_pkt.get_payload<uint32_t>();
	int n_chips = payload32.size();

	if (n_chips > CHIP_TOT || (n_chips == 1 && payload32[0] == 0xffffffff)){
		std::cout << "scan failed" << std::endl;
		return std::vector<uint32_t>();
	}

	uint8_t stave_id = 15;
	int index = 0;

	// print formatted scan
	for(auto id : payload32){

		uint8_t rcv_stave_id = (id>>8);

		if (rcv_stave_id != stave_id){

			if (stave_id != 15){
				for(int i=index; i<CHIPS_PER_STAVE; i++) std::cout << "-----  ";
			}

			stave_id = rcv_stave_id;
			std::cout << "\nstave id " << (int) stave_id << ":" << std::endl;
			index = 0;
		}

		while (index_to_chipid(index++) != (id&0xff)){
			if (index > CHIPS_PER_STAVE-1) break;
			std::cout << "-----  ";
		}

		std::cout << "0x" << std::setw(3) << std::setfill('0') << std::hex << id << "  ";
	}

	for(int i=index; i<CHIPS_PER_STAVE; i++) std::cout << "-----  ";
	std::cout << std::endl;

	return payload32;
}


uint16_t TDAQBoard::send_alpide_command(uint8_t opcode, uint16_t chip_id,
		uint16_t reg_addr, uint16_t reg_val){

	//int value;
	DataPacket packet;
	packet.push_data(uint8_t(DAQ_SND_ALP_CMD));
	packet.push_data(opcode);
	packet.push_data(chip_id);
	packet.push_data(reg_addr);
	packet.push_data(reg_val);

	comm_driver.write_fifo(packet.build_pkt());

	DataPacket resp_pkt(comm_driver.read_fifo());
	auto resp_payload = resp_pkt.get_payload<uint8_t>();
	uint16_t resp = (resp_payload[1]<<8) | resp_payload[0];

	return resp;
}


uint32_t TDAQBoard::read_fpga_reg(uint16_t reg_addr){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RD_REG));
	packet.push_data(reg_addr);

	comm_driver.write_fifo(packet.build_pkt());

	DataPacket resp_pkt(comm_driver.read_fifo());
	auto resp_payload = resp_pkt.get_payload<uint32_t>();

	return resp_payload[0];
}


int TDAQBoard::write_fpga_reg(uint32_t reg_addr, uint32_t data){

	DataPacket packet;

	packet.push_data(uint8_t(DAQ_WR_REG));
	packet.push_data(reg_addr);
	packet.push_data(data);

	comm_driver.write_fifo(packet.build_pkt());

	return 0;
}



int TDAQBoard::packet_counter(uint8_t* rawdata, uint32_t len){

	// bool, header id (0xffff) was in previous packet
	static uint32_t header_slip = 1;
	// number of words from previous packet
	static uint32_t pkt_remainder = 0;

	if (len%2 != 0)
		throw std::runtime_error("unexpted data lenght: unaligned read");

	uint32_t n = 0;
	uint32_t word_cnt = len/2;

	// check if data contains a single incomplete package segment
	if (pkt_remainder+header_slip > word_cnt){

		if(word_cnt > pkt_remainder)
			throw std::runtime_error("unexpted packet lenght");

		pkt_remainder -= word_cnt;

		if (pkt_remainder == 0)
			packet_count++;

		return 0;
	}

	uint16_t* rawdata16 = reinterpret_cast<uint16_t*>(rawdata);

	// count packets
	while(n < word_cnt){

		// get packet lenght
		uint32_t len;
		len = (n==0) ? rawdata16[pkt_remainder+header_slip] :
			rawdata16[n+header_slip];

		// move index to next header
		// header is 6 words (5 + 0xffff)
		n += pkt_remainder + len/2 + 5 + header_slip;
		header_slip = 1;


		if (pkt_remainder != 0)
			packet_count++;

		if (n>word_cnt){
			pkt_remainder = n-word_cnt;
		}
		else {
			pkt_remainder = 0;
			packet_count++;
		}


		// check if last word in data buffer is a packet header
		if (n == word_cnt-1) {

			if (rawdata16[n] != 0xffff)
				throw std::runtime_error("unexpted packet lenght");

			header_slip = 0;
			pkt_remainder = 0;
			break;
		}

	}

	return 0;
}


int TDAQBoard::readout_loop(uint32_t meta,
		uint32_t stop_after_npkts,
		uint32_t self_trig_freq,
		std::string fname){

	std::ofstream outstrm(fname, std::ios::out | std::ios::trunc | std::ios::binary);
	if (!outstrm.is_open())
		throw std::runtime_error("Can't open file for write");

	// write file header
	outstrm.write(reinterpret_cast<const char *>(&meta), sizeof(meta));

	if (self_trig_freq != 0){
		set_trigger_gen(self_trig_freq);
		set_start_daq(true);
	}
	else if (stop_after_npkts==0){
		set_start_daq(true);
	}


	packet_count = 0;
	int pkt_counter_state = 0;
	std::cout << "readout started.." << std::endl;

	while(!force_terminate){
		// read data
		std::vector<uint8_t> rawdata = comm_driver.read_fifo();

		if (rawdata.size() > 0){
			// count packets
			if (pkt_counter_state == 0)
				pkt_counter_state = packet_counter(rawdata.data(), rawdata.size());

			// write to file
			outstrm.write((char*)&rawdata[0], rawdata.size());
			outstrm.flush();
		}

		// print progress
		if (stop_after_npkts > 0){
			std::cout << "\rProgress: " << std::setprecision(1) << std::fixed <<
				((double)packet_count/stop_after_npkts*100) << "%" << std::flush;
		} else {
			std::cout << "\rpkt_read: " << packet_count << std::flush;
		}

		// stop when stop_after_npkts is set
		if (packet_count >= stop_after_npkts && stop_after_npkts > 0)
			break;

		// terminate/stall condition
		if (rawdata.size() <= 0) {
			if (terminate && !self_trigger_running && !ext_trigger_running)
				break;
			usleep(1000);
		}

		if (terminate){
			if (self_trigger_running)
				set_trigger_gen(0);
			if (ext_trigger_running)
				set_start_daq(false);
		}

		rawdata.clear();
	}

	outstrm.close();

	set_start_daq(false);
	std::cout << "\nTot packets read: " << std::dec << packet_count << std::endl;
	return 0;
}


int TDAQBoard::set_start_daq(bool start){

	DataPacket packet;

	if (start){
		packet.push_data(uint8_t(0xfe));
		ext_trigger_running = true;
	}
	else{
		packet.push_data(uint8_t(0xfd));
		ext_trigger_running = false;
	}

	comm_driver.write_fifo(packet.build_pkt());

	return 0;
}


int TDAQBoard::set_trigger_gen(int rate){

	// disable trig generator
	if (rate == 0){
		write_fpga_reg(0x4, 0);
		self_trigger_running = false;
		return 0;
	}

	// divider is max 16 bits and scaled by 0x10 = 16
	uint16_t divider = (40000000/(rate*16)) & 0xffff;
	printf("trigger generator enabled with frequency: %d Hz\n",
			40000000/(divider*16));

	uint32_t reg_value = (0x1<<16) | divider;
	write_fpga_reg(0x4, reg_value);

	self_trigger_running = true;

	return 0;
}


int TDAQBoard::send_pulse(uint8_t stave_id, uint8_t pulses){

	DataPacket packet;
	packet.push_data(uint8_t(0xa));
	packet.push_data(stave_id);
	packet.push_data(pulses);

	comm_driver.write_fifo(packet.build_pkt());

	readout_loop(TYPE_DAQ, pulses);

	return 0;
}


int TDAQBoard::format_flash(){

	std::cout << "formatting flash..." << std::endl;
	DataPacket packet;
	packet.push_data(uint8_t(DAQ_FORMAT_FLASH));

	comm_driver.write_fifo(packet.build_pkt());

	DataPacket response(comm_driver.read_fifo());
	if (response.is_status_reply() == 0)
		std::cout << "done" << std::endl;
	else
		std::cout << "error" << std::endl;

	return 0;
}


std::vector<uint8_t> TDAQBoard::read_flash_file(uint8_t f_index){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RD_FLASH_FILE));
	packet.push_data(f_index);
	comm_driver.write_fifo(packet.build_pkt());

	DataPacket response(comm_driver.read_fifo());
	return response.get_payload<uint8_t>();
}


int TDAQBoard::write_flash_file(uint8_t f_index, std::vector<uint8_t> data){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_WR_FLASH_FILE));
	packet.push_data(f_index);
	packet.push_data(uint16_t(data.size()));
	packet.push_byte_vec(data);

	comm_driver.write_fifo(packet.build_pkt());

	DataPacket response(comm_driver.read_fifo());
	response.is_status_reply();
	return 0;
}


int TDAQBoard::truncate_flash_file(uint8_t f_index){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_WR_FLASH_FILE));
	packet.push_data(f_index);
	packet.push_data(uint8_t(0));
	packet.push_data(uint8_t(0));

	comm_driver.write_fifo(packet.build_pkt());

	DataPacket response(comm_driver.read_fifo());
	return response.is_status_reply();
}


int TDAQBoard::run_hotpix_scan(uint8_t stave_id, uint32_t n_triggers, uint8_t hot_thresh){

	if (hot_thresh > n_triggers){
		printf("invalid options: num triggers < hot threhsold\n");
		return -1;
	}

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RUN_HOTPIXSCN));
	packet.push_data(stave_id);
	packet.push_data(hot_thresh);
	packet.push_data(n_triggers);

	comm_driver.write_fifo(packet.build_pkt());
	std::cout << "hotpix scan started.." << std::endl;

	DataPacket response(comm_driver.read_fifo());
	std::cout << "done" << std::endl;

	return response.is_status_reply();
}


int TDAQBoard::fifo_test(uint16_t chip_id){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RUN_FF_TEST));
	packet.push_data(chip_id);
	comm_driver.write_fifo(packet.build_pkt());

	DataPacket resp_pkt(comm_driver.read_fifo());

	auto resp_payload = resp_pkt.get_payload<uint32_t>();
	uint32_t errors =  resp_payload[0];

	if(errors)
		printf("FIFO test failed for id %#x, erros: %d/4064\n", chip_id, errors);
	else
		printf("FIFO test ok for id %#x\n", chip_id);

	return errors;

}


int TDAQBoard::run_threshscan(uint16_t chip_id, uint8_t nsteps){

	//TODO: deleteme
	uint8_t ninj = 20;

	if (is_gen_brdcast(chip_id)){
		std::cout << "Can't boradcast on all staves" << std::endl;
		return -1;
	}

	int index = chipid_to_index(chip_id);
	int index_max = index;

	if (is_stave_brdcast(chip_id)){
		index = (chip_id>>8) * 10;
		index_max = index+9;
	}

	// serialize scan (only one chip at time)
	for(int i=index; i<=index_max; i++){
		chip_id = index_to_chipid(i);
		std::cout << "Scanning chip " << std::hex << "0x" << chip_id << std::endl;

		DataPacket packet;
		packet.push_data(uint8_t(DAQ_RUN_THRSCN));
		packet.push_data(uint8_t(1));
		packet.push_data(chip_id);
		packet.push_data(nsteps);
		comm_driver.write_fifo(packet.build_pkt());

		std::stringstream stringStream;
		stringStream << "thr_0x" << std::hex << chip_id << ".raw";

		uint32_t meta = TYPE_THRSCAN | (nsteps << 8) | (ninj << 16);
		readout_loop(meta, THRSH_PKTS_PER_STEP*nsteps, 0, stringStream.str());
	}

	return 0;
}


int TDAQBoard::run_threshscan_short(uint16_t chip_id, uint8_t nsteps){

	//TODO: deleteme
	uint8_t ninj = 20;

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RUN_DIGISCAN));
	packet.push_data(uint8_t(0));
	packet.push_data(chip_id);
	comm_driver.write_fifo(packet.build_pkt());

	std::stringstream stringStream;
	stringStream << "thr_0x" << std::hex << chip_id << ".raw";

	uint32_t meta = TYPE_THRSCAN | (nsteps << 8) | (ninj << 16);
	readout_loop(meta, THRSH_PKTS_PER_STEP*nsteps, 0, stringStream.str());

	return 0;
}


int TDAQBoard::run_digiscan(char mode_flag, uint16_t chip_id){

	uint8_t mode = (mode_flag=='d') ? 0x1 : 0;

	std::stringstream stringStream;
	if (mode == 0x1)
		stringStream << "digiscn_0x" << std::hex << chip_id << ".raw";
	else
		stringStream << "white_0x" << std::hex << chip_id << ".raw";

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RUN_DIGISCAN));
	packet.push_data(mode);
	packet.push_data(chip_id);
	comm_driver.write_fifo(packet.build_pkt());

	readout_loop(TYPE_DIGISCAN, DIGIS_PKT_NUM, 0, stringStream.str());
	return 0;
}


int TDAQBoard::read_temp(uint8_t stave_id){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RD_TEMP));
	packet.push_data(stave_id);
	comm_driver.write_fifo(packet.build_pkt());

	DataPacket resp_pkt(comm_driver.read_fifo());
	std::vector<uint32_t> payload32 = resp_pkt.get_payload<uint32_t>();

	if (payload32.size() == 1 && payload32[0] == 0xffffffff){
		std::cout << "failed to read temperature" << std::endl;
		return -1;
	}

	for(uint32_t stemp: payload32){
		float temp;
		std::memcpy(&temp, &stemp, sizeof(float));
		std::cout << temp << " C" << std::endl;
	}

	return 0;
}



int TDAQBoard::read_alp_temp(uint8_t stave_id){

	DataPacket packet;
	packet.push_data(uint8_t(DAQ_RD_ALP_TEMP));
	packet.push_data(stave_id);
	comm_driver.write_fifo(packet.build_pkt());

	DataPacket resp_pkt(comm_driver.read_fifo());
	std::vector<uint16_t> payload16 = resp_pkt.get_payload<uint16_t>();

	if (payload16.size() < 10){
		std::cout << "Error" << std::endl;
		return -1;
	}

	uint16_t start = stave_id*10;
	for (int i=0; i<10; i++)
		std::cout << "id: 0x" <<  std::hex << index_to_chipid(i+start)
			<<" raw temp: 0x" << payload16[i] << std::endl;

	return 0;
}


int TDAQBoard::fpga_reset(){
	DataPacket packet;
	packet.push_data(uint8_t(0xff));

	for(int tries=1; tries<3; tries++){
		std::cout << "reset try " << tries << "/3..." << std::endl;
		comm_driver.write_fifo(packet.build_pkt());
		sleep(1);
		if( ping({0xabcd1234}, true) ){
			std::cout << "reset successful" << std::endl;
			return 0;
		}
	}

	std::cout << "reset fail" << std::endl;
	return 1;
}


int TDAQBoard::set_tsp_switch(uint8_t stave_id, uint8_t channel, uint8_t setrst){

	DataPacket packet;

	packet.push_data(uint8_t(0x17));
	packet.push_data(stave_id);
	packet.push_data(channel);
	packet.push_data(setrst);

	comm_driver.write_fifo(packet.build_pkt());

	return 0;
}


int TDAQBoard::set_tsp_poweron(bool enable){

	DataPacket packet;

	if (enable)
		packet.push_data(uint8_t(0x19));
	else
		packet.push_data(uint8_t(0x18));

	comm_driver.write_fifo(packet.build_pkt());

	sleep(2);

	uint32_t res = read_fpga_reg(0x9);
	if (res == 0xffffffff){
		std::cerr << "Command sent, but error reading power status register" << std::endl;
		return -1;
	}

	for(int i=0; i<3; i++){

		if ((res&0x1) == 1)
			std::cout << "channel " << i << " OK" << std::endl;
		else
			std::cout << "channel " << i << " Fail" << std::endl;

		res = res >> 1;
	}

	return 0;
}


// *********************************************************************************** //

int config_handler(void* user, const char* section,
		const char* name,	const char* value){

	uint16_t chip_id = strtoul(section, NULL, 16);
	uint16_t reg_value = strtoul(value, NULL, 16);

	std::map<std::string, uint16_t> reg_addr {
		{"mode_control", ALPIDE_REG_MODE_CTRL},
		{"CMU_DMU",      ALPIDE_REG_CMU_DMU},
		{"FROMU_conf_1", ALPIDE_REG_FROMU1},
		{"FROMU_conf_2", ALPIDE_REG_FROMU2},
		{"FROMU_conf_3", ALPIDE_REG_FROMU3},
		{"FROMU_pulsing_1", ALPIDE_REG_PULSING1},
		{"FROMU_pulsing_2", ALPIDE_REG_PULSING2},
		{"VRESETP", ALPIDE_REG_VRESETP},
		{"VRESETD", ALPIDE_REG_VRESETD},
		{"VCASP",   ALPIDE_REG_VCASP},
		{"VCASN",   ALPIDE_REG_VCASN},
		{"VCASN2",  ALPIDE_REG_VCASN2},
		{"VCLIP",   ALPIDE_REG_VCLIP},
		{"VTEMP",   ALPIDE_REG_VTEMP},
		{"IAUX2",   ALPIDE_REG_IAUX2},
		{"IRESET",  ALPIDE_REG_IRESET},
		{"IDB",     ALPIDE_REG_IDB},
		{"IBIAS",   ALPIDE_REG_IBIAS},
		{"ITHR",    ALPIDE_REG_ITHR}
	};

	auto search = reg_addr.find(name);
	if ( search == reg_addr.end()){
		std::cerr << "Warning: invalid conf key found: " <<
			name << std::endl;
		return 0; // err
	}

	std::vector<uint16_t> *conf_buffer =
		static_cast<std::vector<uint16_t> *>(user);

	conf_buffer->push_back(chip_id);
	conf_buffer->push_back(reg_addr[name]);
	conf_buffer->push_back(reg_value);
	return 1; // ok
}


int read_config(TDAQBoard &tdaq, std::string fname, bool flash){

	std::vector<uint16_t> config_buffer;
	ini_parse(fname.c_str(), config_handler, &config_buffer);

	if (flash){

		std::vector<uint8_t> buff8_cpy(config_buffer.size()*2);
		std::memcpy(buff8_cpy.data(), config_buffer.data(), config_buffer.size()*2);
		tdaq.write_flash_file(0, buff8_cpy);

	}
	else{

		for(uint32_t n=0; n<config_buffer.size(); n+=3)
			tdaq.send_alpide_command(ALPIDE_WROP,
					config_buffer[n], config_buffer[n+1], config_buffer[n+2]);

	}

	return 0;
}


void dump_chip_registers(TDAQBoard &tdaq, uint16_t chip_id){

	// dump conf regs
	for (uint16_t reg=1; reg<=0x1b; reg++){
		uint16_t value = tdaq.send_alpide_command(ALPIDE_RDOP, chip_id, reg, 0);
		std::cout << std::hex << "0x" << chip_id << " 0x" << reg << " 0x" << value
			<< std::endl;
	}

	// dump daq conf regs
	for (uint16_t reg=0x600; reg<=0x611; reg++){
		uint16_t value = tdaq.send_alpide_command(ALPIDE_RDOP, chip_id, reg, 0);
		std::cout << std::hex << "0x" << chip_id << " 0x" << reg << " 0x" << value
			<< std::endl;
	}

}


void dump_registers(TDAQBoard &tdaq, uint16_t chip_id){

	int nchip, startindex;

	if (is_gen_brdcast(chip_id)){
		startindex = 0;
		nchip = STAVES_N * CHIPS_PER_STAVE;
	}
	else if (is_stave_brdcast(chip_id)){
		startindex = (chip_id>>8)*10;
		nchip = CHIPS_PER_STAVE;
	}
	else
		nchip =1;

	if (nchip == 1){
		dump_chip_registers(tdaq, chip_id);
		return;
	}

	for (int index=startindex; index<startindex+nchip; index++){
		chip_id = index_to_chipid(index);
		dump_chip_registers(tdaq, chip_id);
	}

}


uint16_t column_pixreg_addr(int x){
	return 0x0400 | (x&0x3e0)<<6 | (1 + (x>>4&0x1));
}
uint8_t reverse(uint8_t b) {
	return (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
}

int pbm2pulsemap(TDAQBoard &tdaq, uint32_t chip_id, const char *fname){

	FILE *pic;
	pic = fopen(fname, "r");
	if (pic == NULL){
		printf("Can't open file %s\n", fname);
		return -1;
	}

	int read_size;
	uint8_t buff[80000];
	read_size = fread(buff, 1, 80000, pic);
	fclose(pic);

	// check header
	if(buff[0] != 0x50 || buff[1] != 0x34){
		printf("bad header\n");
		return -1;
	}

	// skip comments
	int offset = 3;
	if (buff[3] == 0x23){
		while(buff[offset] != 0x0a && buff[offset+1] != 0x23){
			offset++;
			if (offset == read_size)
				return -1;
		}
	}
	offset++;

	// skip resolution
	while(buff[offset] != 0x0a)
		offset++;
	offset++;

	// add reset pixregs?

	// pulse data
	tdaq.send_alpide_command(ALPIDE_WROP, chip_id, ALPIDE_REG_PIXCFG, 0x3);

	uint16_t col_addr, col_sel, row_addr, row_sel;
	for (int x=0; x<1024; ++x){
		// column select
		col_addr = column_pixreg_addr(x);
		col_sel  = 1<<(x&0xf);
		for (int yr=0; yr<32; ++yr){
			if (terminate) return 0;
			if (buff[offset] == 0 && buff[offset+1] == 0){
				offset += 2;
				continue;
			}
			tdaq.send_alpide_command(ALPIDE_WROP, chip_id, col_addr, col_sel);
			// row select
			row_addr = yr<<11 | 0x404;
			row_sel = reverse(buff[offset+1])<<8 | reverse(buff[offset]);
			offset += 2;
			tdaq.send_alpide_command(ALPIDE_WROP, chip_id, row_addr, row_sel);
			// clear selection
			tdaq.send_alpide_command(ALPIDE_WROP, chip_id, 0x487, 0x0);
		}
		printf("col: %i\r", x);
		fflush(stdout);
	}
	printf("\n");

	return 0;
}


int print_config_file(std::vector<uint8_t> data){

	std::cout << "Config file dump:" << std::endl;

	std::vector<uint16_t> payload16_cpy(data.size()/2);
	std::memcpy(payload16_cpy.data(), data.data(), data.size());

	if (payload16_cpy[0] == 0xffff && payload16_cpy[1] == 0xffff){
		std::cout << "Config file is empty" << std::endl;
		return 0;
	}

	for (uint32_t n=0; n<payload16_cpy.size(); n+=3)
		std::cout << std::hex <<
			" 0x" << payload16_cpy[n] <<
			" 0x" << payload16_cpy[n+1] <<
			" 0x" << payload16_cpy[n+2] << std::endl;

	return 0;

}

int print_hotpix_file(std::vector<uint8_t> data){

	if ( data.size() == 0){
		std::cout << "Hot pixels file is empty" << std::endl;
		return 0;
	}

	std::cout << "Hot pixel list:" << std::endl;
	std::vector<uint32_t> payload32_cpy(data.size()/4);
	std::memcpy(payload32_cpy.data(), data.data(), data.size());

	if (payload32_cpy[0] == 0xffffffff){
		std::cout << "hotpix file empty" << std::endl;
		return 0;
	}

	for (auto x: payload32_cpy){
		uint32_t word = x;
		uint8_t staveid = word>>24;
		uint8_t id_lsb  = (word>>20)&0xf;
		uint16_t pix_y  = (word>>10)&0x3ff;
		uint16_t pix_x  = word&0x3ff;

		uint16_t chip_id = 0x70 | (staveid<<8) | id_lsb;

		printf("chip_id: %#05x   col,row: %i,%i\n", chip_id, pix_x, pix_y);

	}

	return 0;

}


std::vector<uint16_t> get_chipid_list(int stave_no){
	std::vector<uint16_t> chip_id_list;

	for(int i=0x70; i<=0x74; i++){
		uint16_t chip_id = i | (stave_no<<8);
		chip_id_list.push_back(chip_id);
	}

	for(int i=0x78; i<=0x7c; i++){
		uint16_t chip_id = i | (stave_no<<8);
		chip_id_list.push_back(chip_id);
	}

	return chip_id_list;
}


int run_digiscan_ser(TDAQBoard &tdaq, char mode, uint16_t chip_id){

	if (is_gen_brdcast(chip_id)){
		printf("Stave broadcast address not accepted for this test\n");
		return -1;
	}

	if (is_stave_brdcast(chip_id)){
		std::vector<uint16_t> chip_id_list = get_chipid_list((chip_id>>8));

		for(auto id: chip_id_list){
			std::cout << "scanning id 0x" << std::hex << id << ": ";
			tdaq.run_digiscan(mode, id);
		}

	}
	else{
		std::cout << "scanning id 0x" << std::hex << chip_id << ": ";
		tdaq.run_digiscan(mode, chip_id);
	}

	return 0;
}


// ***************** Dumb parser ***************
bool optstrcmp(std::string shortopt, std::string longopt, std::string input){
	return (shortopt == input || longopt == input);
}

template <typename T>
T get_arg(std::vector<std::string> &args, bool optional=false, T defaultv=T()) {

	T ret;

	if (args.size() == 0){
		if(optional)
			return defaultv;
		else
			throw std::runtime_error("Missing arguments");
	}

	std::string curr_arg = args.front();
	if (curr_arg.rfind("-") == 0 || curr_arg.rfind("--") == 0) {
		if(optional)
			return defaultv;
		else
			throw std::runtime_error("Missing arguments");
	}

	std::istringstream iss(curr_arg);

	if (curr_arg.rfind("0x") == 0)
		iss >> std::hex >> ret;
	else if (curr_arg.find_first_not_of("0123456789") == std::string::npos)
		iss >> std::dec >> ret;
	else
		iss >> ret;

	args.erase(args.begin());
	return ret;
}

uint16_t StaveID_FromString(std::string in){
	int b1=in.find('_');
	int b2=in.find('_',b1+1);

	if(b1>0 && b2<0 ){
		//staveid separate format
		std::string sturr=in.substr(0,b1);
		std::string sstav=in.substr(b1+1);
		uint16_t turret=strtoll(sturr.c_str(),0,0);
		uint16_t stave =strtol(sstav.c_str(),0,0);

		if (turret <=4 && stave <=2)
			return ((turret*3)+stave);
	}else if(b1<0){
		//chipid single format
		uint16_t stave = strtol(in.c_str(),0,0);
		if(stave <=16) return stave;
	}
	printf("StaveID format non valid\n");
	return 0xFFFF;
}

uint16_t ChipID_FromString(std::string in){
	int b1=in.find('_');
	int b2=in.find('_',b1+1);
	int b3=in.find('_',b2+1);
	if(b1>0 && b2>0 && b3<0){
		//chipid separate format
		std::string sturr=in.substr(0,b1);
		std::string sstav=in.substr(b1+1,b2-b1-1);
		std::string schip="0x"+in.substr(b2+1);
		uint16_t turret=strtoll(sturr.c_str(),0,0);
		uint16_t stave =strtol(sstav.c_str(),0,0);
		uint16_t chipID =strtol(schip.c_str(),0,0);
		if (turret <=4 && stave <=2 &&
				((chipID>=0x70 && chipID<=0x74) || (chipID>=0x78 && chipID<=0x7C)|| chipID==0x0F)
			){
			uint16_t out=((turret*3)+stave)<<8;
			out|=chipID;
			return out;
		}
	}else if(b1<0){
		//chipid single format
		int cc= strtol(in.c_str(),0,0);
		uint16_t stave = cc>>8;
		uint16_t chipID = cc&0xFF;

		if(stave <=16 &&
				((chipID>=0x70 && chipID<=0x74) || (chipID>=0x78 && chipID<=0x7C)|| chipID==0x0F)
			)
			return cc;

	}
	printf("ChipID format non valid\n");
	return 0xFFFF;
}


int main(int argc, char *argv[]){

	signal(SIGINT, sigint_handler);

	const char helpstr[] =
		"-h --help\n"
		"    print helpstring\n"
		"\n"
		"====================      NOTE ON CHIPD ID and STAVE ID"
		"\n"
		" Chipid  8bit chipID | 4-MSB=0x7 | bit3=chip column [0-1]| 3LSB= O: master, [1,2,3,4] slaves|\n"
		" ChipIDs on the single stave are:\n"
		" 		First  line: 0x70(master), 0x71, 0x72, 0x73, 0x74\n"
		" 		Second line: 0x78(master), 0x79, 0x7A, 0x7B, 0x7C\n"
		" The ChipID 0x0F is a broadcast adddress for all the chips on a single stave\n"
		"\n"
		" Extended  16bit chipID |8bit = stave number| 8bit chipID|\n"
		" stave number goes from 0x0 to 0xE, with 0xF as Broadcast address for staves\n"
		" The full broadcast address id 0xF0F\n"
		"\n"
		" The following commands accept also a <chip_id> expressed as Turret_Stave_chipid\n"
		" where Turret [0-5] stave 0=top 1=middle 2=bottom, an the 8bit chip id in Hex\n"
		" Ex: 2_0_7A  = turret 2 (central) stave 0(top) chip 7A (second line and second slave) \n "
		"\n"
		" Also <stave_id> could be a integer 0-14 or in the form  Turret_Stave\n"
		"\n"
		"=========  TDAQ control\n"
		"-0 --reset-fpga\n"
		"    send soft-reset command to fpga\n"
		"   --fpga-write <reg_addr> <value>\n"
		"    write fpga control register\n"
		"   --fpga-read <reg_addr>\n"
		"    read fpga control register\n"
		"-p --ping <hex_data>\n"
		"   fpga echo test <hex_data> is a series of hex strings 0xabcd1234\n"
		"====================================================================\n"
		"\n"
		"=========  TSP POWER control\n"
		"--set-tsp-switch <stave_id> <channel> <status>\n"
		"    channel 0:bias 1:analog 2:digital\n"
		"    status  0:off 1:on\n"
		" MISSING COMMAND --check-tsp-switch <stave_id>\n"
		"    returns the status of the switches for the requested stave\n"
		"--set-tsp-pwron\n"
		"    power on all staves\n"
		"--set-tsp-pwroff\n"
		"    power off all staves\n"
		"====================================================================\n"
		"\n"
		"=========  ALPIDE configuration\n"
		"-i --init\n"
		"    init alpide chips (broadcast)\n"
		"-c --config <file.cfg>\n"
		"    read alpide config file and apply the content to the tracker\n"
		"--write-cfg-file <file.cfg>\n"
		"    write alpide config file to flash\n"
		"====================================================================\n"
		"\n"
		"=========  TDAQ FLASH MEMORY interface\n"
		"--list-files\n"
		"    list files in the flash\n"
		"--format-flash\n"
		"    format the flash\n"
		"--read-file <index>\n"
		"    read file with index <index> from the flash\n"
		"--truncate-file <index>\n"
		"    truncate to zero size the file with index <index> on the flash\n"
		"MISSING COMMAND --write-file <index> <file.cfg>\n"
		"    write  file to the flash at the position indicate by index \n"

		"====================================================================\n"
		"\n"
		"=========  ALPIDE TEST section\n"
		"-a --chip-scan\n"
		"    scan for connected chips, report found chip IDs\n"

		"\n"

		"--fifotest  <chip_id>\n"
		"    run fifotest on chipid\n"
		"-r --read-reg <chip_id> <reg_addr>\n"
		"    read alpide register\n"
		"-w --write-reg  <chip_id> <reg_addr> <value>\n"
		"    write alpide register\n"
		"--dump-regs  <chip_id>\n"
		"    dump all registers to stdout, broadcast address is allowed\n"
		"-T --thrscan  <chip_id>\n"
		"    run thresholdscan on chip_id\n"
		"-D --digiscan  <chip_id>\n"
		"    run digital scan on chipid\n"
		"-W --whitescan  <chip_id>\n"
		"    run white frame scan on chipid\n"
		"--thrscan-short <mode>  <chip_id>\n"
		"    same as thrscan but test is run on a small area\n"
		"-S --hotpix-scan <stave_id> [n_trigs] [threshold]\n"
		"    scan for hot pixels, result is saved in flash file index 1\n"
		"-A --all-tests <stave_id>\n"
		"    run fifo test and threshold scan on every chip on stave\n"
		"======================================================================================================\n"
		" ========================  CHIP MASKING  \n"
		"--set-chipmask <stave_id> <10-bit hex value>\n"
		"    set chipmask for stave <stave_id>, bit in the mask follow the order of chipid numbers\n"
		"--read-chipmask <stave_id>\n"
		"    read chipmask for stave <stave_id>\n"
		"--mask-chip  <chip_id>\n"
		"    mask the given chip id\n"
		"========================================================\n"
		"\n"
		"=========================  DAQ Control\n"
		"-q --run-daq\n"
		"-m --run-daq-trig [trig freq]\n"
		"    enable trigger generator and start DAQ, default frequency is 1000 Hz\n"
		"--pulse <stave_id> <n>\n"
		"    broadcast <n> pulses to <stave_id> and start readout\n"
		"\n"
		"===========================   MISC \n"
		"--read-temp <stave_id>\n"
		"    read temperature\n"
		"--read-alp-temp <stave_id>\n"
		"    read raw temperature from the chips in stave no.<stave_id>\n"
		"--pbm2pulsemap <stave_id> <chipid> <pic.pbm>\n"
		"    load pulsemap from given .pbm file\n";


	if (argc < 2){
		std::cerr << "No command selected" << std::endl;
		std::cout << helpstr;
		return -1;
	}

	TDAQBoard tdaq;

	//TODO: real parser
	std::string verb_option = argv[1];
	std::vector<std::string> args_p(&argv[2], &argv[2]+argc-2);

	if (optstrcmp("-h", "--help", verb_option)){
		std::cout << helpstr;
	}
	else if (optstrcmp("-i", "--init", verb_option)){
		tdaq.run_alp_init();
	}
	else if (optstrcmp("-c", "--config", verb_option)){
		std::string asd = get_arg<std::string>(args_p);
		read_config(tdaq, asd, false);
	}
	else if (optstrcmp("", "--write-cfg-file", verb_option)){
		read_config(tdaq, get_arg<std::string>(args_p), true);
	}
	else if (optstrcmp("-a", "--chip-scan", verb_option)){
		tdaq.chipscan();
	}
	else if (optstrcmp("-p", "--ping", verb_option)){
		std::vector<uint32_t> data;
		for(int n=2; n<argc; n++)
			data.push_back(get_arg<uint32_t>(args_p));
		tdaq.ping(data);
	}
	else if (optstrcmp("-T", "--thrscan", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		uint8_t nsteps = get_arg<uint16_t>(args_p);
		if(chip_id==0xFFFF) return -3;
		tdaq.run_threshscan(chip_id, nsteps);
	}
	else if (optstrcmp("", "--thrscan-short", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		uint8_t nsteps = get_arg<uint16_t>(args_p);
		if(chip_id==0xFFFF) return -3;
		tdaq.run_threshscan_short(chip_id, nsteps);
	}
	else if (optstrcmp("-D", "--digiscan", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		run_digiscan_ser(tdaq, 'd', chip_id);
	}
	else if (optstrcmp("-W", "--whitescan", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		run_digiscan_ser(tdaq, 'w', chip_id);
	}
	else if (optstrcmp("-w", "--write-reg", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		uint16_t reg_addr = get_arg<uint16_t>(args_p);
		uint16_t reg_val  = get_arg<uint16_t>(args_p);
		tdaq.send_alpide_command(ALPIDE_WROP, chip_id, reg_addr, reg_val);
		std::cout << std::hex << "write chipid 0x" << chip_id << " reg 0x"
			<< reg_addr << std::endl;
	}
	else if (optstrcmp("-r", "--read-reg", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		uint16_t reg_addr = get_arg<uint16_t>(args_p);
		uint16_t value = tdaq.send_alpide_command(ALPIDE_RDOP, chip_id, reg_addr, 0);
		std::cout << std::hex << "read chipid 0x" << chip_id << " reg 0x" << reg_addr
			<< " value 0x" << value << std::endl;
	}
	else if (optstrcmp("", "--set-chipmask", verb_option)){
		uint8_t  stave_id = StaveID_FromString(get_arg<std::string>(args_p));
		uint16_t chipmask = get_arg<uint16_t>(args_p);
		uint16_t chip_id = (stave_id << 8);
		tdaq.send_alpide_command(FSM_SET_MSK, chip_id, 0, chipmask);
	}
	else if (optstrcmp("", "--read-chipmask", verb_option)){
		uint8_t  stave_id = StaveID_FromString(get_arg<std::string>(args_p));
		uint16_t chip_id = (stave_id<<8);
		uint16_t value = tdaq.send_alpide_command(FSM_RD_MSK, chip_id, 0, 0);

		char masked_list[CHIPMASK_STRING_LEN] = "0\0";
		int num = printable_chipmask(value, masked_list);
		printf("%d masked chips in stave %d:\n", num, stave_id);
		printf("%s\n", masked_list);
	}
	else if (optstrcmp("", "--mask-chip", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		uint16_t chipmask = tdaq.send_alpide_command(FSM_RD_MSK, chip_id, 0, 0);
		uint16_t index = chipid_to_index(chip_id);
		chipmask |= (1<<index);
		tdaq.send_alpide_command(FSM_SET_MSK, chip_id, 0, chipmask);
	}
	else if (optstrcmp("-q", "--run-daq", verb_option)){
		tdaq.readout_loop(TYPE_DAQ);
	}
	else if (optstrcmp("-m", "--run-daq-trig", verb_option)){
		tdaq.readout_loop(TYPE_DAQ, 0, get_arg<int>(args_p, true, 1000));
	}
	else if (optstrcmp("-S", "--hotpix-scan", verb_option)){
		uint8_t  stave_id = get_arg<uint16_t>(args_p);
		uint32_t n_triggers = get_arg<uint32_t>(args_p, true, 10000);
		uint32_t thresh = get_arg<uint32_t>(args_p, true, 20);
		tdaq.run_hotpix_scan(stave_id, n_triggers, thresh);
	}
	else if (optstrcmp("-0", "--reset-fpga", verb_option)){
		tdaq.fpga_reset();
	}
	else if (optstrcmp("", "--fpga-write", verb_option)){
		uint32_t reg_addr = get_arg<uint32_t>(args_p);
		uint32_t reg_val  = get_arg<uint32_t>(args_p);
		tdaq.write_fpga_reg(reg_addr, reg_val);
	}
	else if (optstrcmp("", "--fpga-read", verb_option)){
		uint32_t reg_addr = get_arg<uint32_t>(args_p);
		uint32_t value = tdaq.read_fpga_reg(reg_addr);
		std::cout << std::hex << "fpga reg 0x" << reg_addr <<
			" value 0x" << value << std::endl;
	}
	else if (optstrcmp("", "--fifotest", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		tdaq.fifo_test(chip_id);
	}
	else if (optstrcmp("", "--pulse", verb_option)){
		uint8_t stave_id =StaveID_FromString(get_arg<std::string>(args_p));
		uint8_t pulses = get_arg<uint16_t>(args_p);
		tdaq.send_pulse(stave_id, pulses);
	}
	else if (optstrcmp("", "--read-file", verb_option)){
		std::uint8_t index = get_arg<uint16_t>(args_p);
		std::vector<uint8_t> data = tdaq.read_flash_file(index);
		if (index == 0)
			print_config_file(data);
		else if (index == 1)
			print_hotpix_file(data);
	}
	else if (optstrcmp("", "--truncate-file", verb_option)){
		tdaq.truncate_flash_file(get_arg<uint16_t>(args_p));
	}
	else if (optstrcmp("", "--format-flash", verb_option)){
		tdaq.format_flash();
	}
	else if (optstrcmp("", "--dump-regs", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		dump_registers(tdaq, chip_id);
	}
	else if (optstrcmp("", "--read-temp", verb_option)){
		uint16_t stave_id  = StaveID_FromString(get_arg<std::string>(args_p));
		tdaq.read_temp(stave_id);
	}
	else if (optstrcmp("", "--read-alp-temp", verb_option)){
		uint16_t stave_id  = StaveID_FromString(get_arg<std::string>(args_p));
		tdaq.read_alp_temp(stave_id);
	}
	else if (optstrcmp("", "--pbm2pulsemap", verb_option)){
		std::string chip_id_string  = get_arg<std::string>(args_p);
		uint16_t chip_id = ChipID_FromString(chip_id_string);
		if(chip_id==0xFFFF) return -3;
		std::string fname = get_arg<std::string>(args_p);
		pbm2pulsemap(tdaq, chip_id, fname.c_str());
	}
	else if (optstrcmp("", "--set-tsp-switch", verb_option)){
		uint8_t stave_id = StaveID_FromString(get_arg<std::string>(args_p));
		uint8_t channel  = get_arg<uint16_t>(args_p);
		uint8_t setrst   = get_arg<uint16_t>(args_p);
		tdaq.set_tsp_switch(stave_id, channel, setrst);
	}
	else if (optstrcmp("", "--set-tsp-pwroff", verb_option)){
		tdaq.set_tsp_poweron(false);
	}
	else if (optstrcmp("", "--set-tsp-pwron", verb_option)){
		tdaq.set_tsp_poweron(true);
	}
	else{
		std::cout << "No command selected" << std::endl;
	}


	return 0;
}
