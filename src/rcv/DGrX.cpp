#include "DGrX_rev_4.hpp"
#include "DGrX_rev_9.hpp"

#ifndef WIN32
#define WIN32
#endif

#include "rtklib.h"

gtime_t latest_time;

enum ReturnCodes {
	end_of_file = -2,
	error_message = -1,
	no_message = 0,
	input_observation_data = 1,
	input_ephemeris = 2,
	input_sbas_message = 3,
	input_ion_utc_parameter = 9,
	input_lex_message = 31
};


namespace rev_4 {
	int decode_position(DGrX_rev_4::MeasuredPositionData *message, raw_t *raw) {
		//std::array<double, 6> gpst0 = { 1980, 1, 6, 0, 0, 0 };
		//raw->time = timeadd(epoch2time(gpst0.data()), min*60.0 + msec*0.001);
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");

			auto& message_data = message->Data();
			if (static_cast<int>(message_data.fix_quality.fix_status)) {
				raw->time = gpst2time(message_data.wn, message_data.rcv_time * 1e-3);
				return ReturnCodes::input_observation_data;
			}
			else
				return ReturnCodes::no_message;
		}
		catch (...) {
			//return ReturnCodes::error_message;

			return ReturnCodes::no_message;
		}
		return ReturnCodes::no_message;
	}

	int decode_raw_data(DGrX_rev_4::RawMeasurementData *message, raw_t *raw) {

		return ReturnCodes::input_observation_data;
	}

	int ConvertToRaw(DGrX_rev_4::Message *message, raw_t *raw) {
		if (message == nullptr || raw == nullptr)
			//return ReturnCodes::error_message;
			return ReturnCodes::no_message;
		auto type = message->GetMID();
		switch (type)
		{
		case DGrX_rev_4::MID::CommandAcknowledgement:
			break;
		case DGrX_rev_4::MID::CommandNAcknowledgement:
			break;
		case DGrX_rev_4::MID::AlmanacStatus:
			break;
		case DGrX_rev_4::MID::DebugData:
			break;
		case DGrX_rev_4::MID::ClockStatus:
			break;
		case DGrX_rev_4::MID::GLONASSEphemerisData:
			break;
		case DGrX_rev_4::MID::LLAOutputMessage:
			break;
		case DGrX_rev_4::MID::GPSEphemerisData:
			break;
		case DGrX_rev_4::MID::RAIMAlertLimit:
			break;
		case DGrX_rev_4::MID::RawMeasurementData:
			return decode_raw_data(dynamic_cast<DGrX_rev_4::RawMeasurementData*>(message), raw);
			break;
		case DGrX_rev_4::MID::ExcludedSV:
			break;
		case DGrX_rev_4::MID::FirmwareSchematicVersion:
			break;
		case DGrX_rev_4::MID::MeasuredPositionData:
			return decode_position(dynamic_cast<DGrX_rev_4::MeasuredPositionData*>(message), raw);
			break;
		default:
			break;
		}

		return 0;
	}
}

namespace rev_9 {
	int decode_position(DGrX_rev_9::MeasuredPositionData *message, raw_t *raw) {
		std::array<double, 6> gpst0 = { 1980, 1, 6, 0, 0, 0 };
		//raw->time = timeadd(epoch2time(gpst0.data()), min*60.0 + msec*0.001);
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");

			auto& message_data = message->Data();
			if (static_cast<int>(message_data.fix_quality.fix_status)) {
				raw->time = timeadd(epoch2time(gpst0.data()), message_data.rcv_time * 0.001);
				//raw->time = gpst2time(message_data.wn, message_data.rcv_time * 1e-3);
				return ReturnCodes::input_observation_data;
			}
			else
				return ReturnCodes::no_message;

		}
		catch (...) {
			//return ReturnCodes::error_message;

			return ReturnCodes::no_message;
		}
		return ReturnCodes::no_message;
	}

	int decode_raw_data(DGrX_rev_9::RawMeasurementData *message, raw_t *raw) {

		return ReturnCodes::no_message;
	}

	int ConvertToRaw(DGrX_rev_9::Message *message, raw_t *raw) {
		if (message == nullptr || raw == nullptr)
			//return ReturnCodes::error_message;
			return ReturnCodes::no_message;
		auto type = message->GetMID();
		switch (type)
		{
		case DGrX_rev_9::MID::CommandAcknowledgement:
			break;
		case DGrX_rev_9::MID::L5E5G3MeasurementData:
			break;
		case DGrX_rev_9::MID::CommandNAcknowledgement:
			break;
		case DGrX_rev_9::MID::GPSAlmanacData:
			break;
		case DGrX_rev_9::MID::ClockStatus:
			break;
		case DGrX_rev_9::MID::GLONASSEphemerisData:
			break;
		case DGrX_rev_9::MID::RinexHeaderInformation:
			break;
		case DGrX_rev_9::MID::GalileoEphemerisData:
			break;
		case DGrX_rev_9::MID::LLAOutputMessage:
			break;
		case DGrX_rev_9::MID::GPSEphemerisData:
			break;
		case DGrX_rev_9::MID::RAIMAlertLimit:
			break;
		case DGrX_rev_9::MID::GlonassAlmanacData:
			break;
		case DGrX_rev_9::MID::RawMeasurementData:
			return decode_raw_data(dynamic_cast<DGrX_rev_9::RawMeasurementData*>(message), raw);
			break;
		case DGrX_rev_9::MID::ExcludedSV:
			break;
		case DGrX_rev_9::MID::FirmwareSchematicVersion:
			break;
		case DGrX_rev_9::MID::MeasuredPositionData:
			return decode_position(dynamic_cast<DGrX_rev_9::MeasuredPositionData*>(message), raw);
			break;
		default:
			break;
		}

		return 0;
	}
}

extern "C" int input_dgrx_4(raw_t *raw, unsigned char data) {
	// not implemented yet
	return error_message;
}

extern "C" int input_dgrx_9(raw_t *raw, unsigned char data) {
	// not implemented yet
	return error_message;
}

extern "C" int input_dgrx_4f(raw_t *raw, FILE *fp) {
	//trace(4, "input_dgr8f:\n");

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

	//auto message = DGrX_rev_4::ReadStruct(log_file);
	return rev_4::ConvertToRaw(DGrX_rev_4::ReadStruct(log_file).get(), raw);
}

extern "C" int input_dgrx_9f(raw_t *raw, FILE *fp) {
	//trace(4, "input_dgr8f:\n");

	std::ifstream log_file(fp);
	std::uint8_t last_byte = 0;
	for (int i = 0;; ++i) {
		log_file >> last_byte;
		if (log_file.eof())
			return -2;
		if(last_byte == 0x44)
			if (DGrX_rev_9::Sync(log_file))
				break;
		if (i >= 4096)
			return 0;

	}
	
	//auto message = DGrX_rev_9::ReadStruct(log_file);
	return rev_9::ConvertToRaw(DGrX_rev_9::ReadStruct(log_file).get(), raw);
}