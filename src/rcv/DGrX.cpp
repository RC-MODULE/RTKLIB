#include "DGrX.hpp"

#if defined(__BORLANDC__)
#if !defined(_WIN64)
#pragma comment(lib, "cw32mti.lib")
#endif
#endif

extern "C" int input_dgrx(raw_t *raw, unsigned char data) {
	try {
		trace(5, "input_dgrx: data=%02x\n", data);
		DataGridTools::GetWnFromSystem();
		if (DataGridTools::byte_sync.EmplaceData(data)) 
			return DataGridTools::ConvertToRaw(DataGridProtocol::ReadStruct(DataGridTools::byte_sync.GetMessageData()).get(), raw);
		
		return DataGridTools::ReturnCodes::no_message;
	}
	catch (...) {
		return DataGridTools::ReturnCodes::error_message;
	}
}

extern "C" int input_dgrxf(raw_t *raw, FILE *fp) {
	try {
		trace(4, "input_dgrxf:\n");
		DataGridTools::GetWnFromFile(fp);

		std::ifstream log_file(fp);
		std::uint8_t last_byte = 0;
		for (int i = 0;; ++i) {
			log_file >> last_byte;
			if (log_file.eof())
				return -2;

			if (last_byte == 0x44)
				if (DataGridProtocol::Sync(log_file))
					break;

			if (i >= 4096)
				return 0;

		}

		return DataGridTools::ConvertToRaw(DataGridProtocol::ReadStruct(log_file).get(), raw, fp);
	}
	catch (...) {
		return DataGridTools::ReturnCodes::error_message;
	}
}
