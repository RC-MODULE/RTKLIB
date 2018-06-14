#include "DGrX.hpp"

//#pragma comment(lib, "bcbie.lib")
//#pragma comment(lib, "bcbsmp.lib")
//#pragma comment(lib, "cp32mt.lib")
#if defined(__BORLANDC__)
#pragma comment(lib, "cp32mti.lib")
#endif
//#pragma comment(lib, "cw32.lib")
//#pragma comment(lib, "cw32i.lib")
//#pragma comment(lib, "cw32mt.lib")
//#pragma comment(lib, "cw32mti.lib")
//#pragma comment(lib, "dxextra.lib")

//#pragma comment(lib, "import32.lib")
//#pragma comment(lib, "mswsock.lib")
//#pragma comment(lib, "obsolete.lib")
//#pragma comment(lib, "ole2w32.lib")
//#pragma comment(lib, "oleaut32.lib")
//#pragma comment(lib, "usebormm.lib")
//#pragma comment(lib, "uuid.lib")
//#pragma comment(lib, "vcllink.lib")
//#pragma comment(lib, "vclstd.lib")
//#pragma comment(lib, "w32inet.lib")
//#pragma comment(lib, "wininet.lib")
//#pragma comment(lib, "ws2_32.lib")


extern "C" int input_dgrx_4(raw_t *raw, unsigned char data) {
	if (DGrX::byte_sync.EmplaceData(data)) {
//		DGrX::vectorwrapbuf<char, unsigned char> v(DGrX::byte_sync.GetMessageData());
//		std::istream is(&v);
//		return DGrX::rev_4::ConvertToRaw(DGrX_rev_4::ReadStruct(is).get(), raw);
		return DGrX::rev_4::ConvertToRaw(DGrX_rev_4::ReadStruct(DGrX::byte_sync.GetMessageData()).get(), raw);
	}
	return DGrX::ReturnCodes::no_message;
	//return error_message;
}

extern "C" int input_dgrx_9(raw_t *raw, unsigned char data) {
	// not implemented yet
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

	return DGrX::rev_4::ConvertToRaw(DGrX_rev_4::ReadStruct(log_file).get(), raw);
}

extern "C" int input_dgrx_9f(raw_t *raw, FILE *fp) {
	//trace(4, "input_dgr8f:\n");
//	GetWnFromFile(fp);
//
//	std::ifstream log_file(fp);
//	std::uint8_t last_byte = 0;
//	for (int i = 0;; ++i) {
//		log_file >> last_byte;
//		if (log_file.eof())
//			return -2;
//		if(last_byte == 0x44)
//			if (DGrX_rev_9::Sync(log_file))
//				break;
//		if (i >= 4096)
//			return 0;
//
//	}
//
//	return rev_9::ConvertToRaw(DGrX_rev_9::ReadStruct(log_file).get(), raw);
	return DGrX::ReturnCodes::error_message;
}