#include "DGrX.hpp"

#if defined(__BORLANDC__)
#if !defined(_WIN64)
#pragma comment(lib, "cp32mti.lib")
#endif
#endif

extern "C" int input_dgrx_4(raw_t *raw, unsigned char data) {
	DGrX::GetWnFromSystem();
	if (DGrX::byte_sync.EmplaceData(data)) {
		return DGrX::rev_4::ConvertToRaw(DGrX_rev_4::ReadStruct(DGrX::byte_sync.GetMessageData()).get(), raw);
	}
	return DGrX::ReturnCodes::no_message;
	//return error_message;
}

extern "C" int input_dgrx_9(raw_t *raw, unsigned char data) {
	return DGrX::ReturnCodes::error_message;
}

extern "C" int input_dgrx_4f(raw_t *raw, FILE *fp) {
	//trace(4, "input_dgr8f:\n");
	DGrX::GetWnFromFile(fp);

	std::ifstream log_file(fp);
	std::uint8_t last_byte = 0;
	for (int i = 0;; ++i) {
		log_file >> last_byte;
		if (log_file.eof())
			return -2;
		if (last_byte == 0x44)
			if (DGrX_rev_4::Sync(log_file))
				break;
		if (i >= 4096)
			return 0;

	}

	return DGrX::rev_4::ConvertToRaw(DGrX_rev_4::ReadStruct(log_file).get(), raw, fp);
}

extern "C" int input_dgrx_9f(raw_t *raw, FILE *fp) {
	return DGrX::ReturnCodes::error_message;
}