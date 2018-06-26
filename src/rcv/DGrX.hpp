#ifndef _DGRx_HPP
#define _DGRx_HPP

#include <chrono>
#include "DGrX_rev_4.hpp"
//#include "DGrX_rev_9.hpp"
#include "..\rtklib.h"

#if defined(__BORLANDC__)
#include <sys/stat.h>
#endif

namespace DGrX {
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

	const double Pi = 4 * std::atan(1.0);

	std::vector<std::int32_t> used_svs;

	std::int32_t whole_1024_weeks = 0;

	struct OverlapCounter final {
	private:
		std::size_t pre = 0;
		std::size_t post = 0;
		std::size_t count = 0;
	public:
		void Update(std::size_t wn) {
			pre = post;
			post = wn;
			if (pre > post)
				++count;
		}

		std::size_t Count() {
			return count;
		}
	} overlap_counter;

	inline double SemicyclesToRadians(double val) {
		return val * Pi;
	}

	inline double RadiansToSemicycles(double val) {
		return val / Pi;
	}

	inline double KilometersToMeters(double val) {
		return val * 1e3;
	}

	gtime_t adjday(const gtime_t &time, double tod) {
		double ep[6], tod_p;
		time2epoch(time, ep);
		tod_p = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
		if (tod < tod_p - 43200.0) tod += 86400.0;
		else if (tod > tod_p + 43200.0) tod -= 86400.0;
		ep[3] = ep[4] = ep[5] = 0.0;
		return timeadd(epoch2time(ep), tod);
	}

	void GetWnFromFile(FILE *fp) {
		if (whole_1024_weeks)
			return;

		struct _stat s;
		auto q = _fstat(_fileno(fp), &s);
		auto time = s.st_mtime;
		gtime_t time_32bit;
		time_32bit.time = time;

		auto week = 0;
		time2gpst(time_32bit, &week);
		whole_1024_weeks = (week / 1024) * 1024;
	}

	void GetWnFromSystem() {
		if (whole_1024_weeks)
			return;

		auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		gtime_t time_32bit;
		time_32bit.time = time;

		auto week = 0;
		time2gpst(time_32bit, &week);
		whole_1024_weeks = (week / 1024) * 1024;
	}

	class ByteSync final {
	private:
		std::array<unsigned char, 4> sync_sequence;
		std::vector<unsigned char> message_data;
		std::size_t current_byte = 0;
		std::size_t message_size = 0;
		bool is_init = false;
		std::unordered_map<DGrX_rev_4::MID, std::size_t> struct_sizes;

	public:
		void Init(){
			sync_sequence = { 'D', 'G', 'R', '8' };
			std::size_t max_size = 0;
			struct_sizes = DGrX_rev_4::GetStructSizes();
			for (auto&el : struct_sizes)
				if (el.second > max_size)
					max_size = el.second;
			max_size += sync_sequence.size() + 1; // message: DGR8{MID}{Message data};
			message_data.reserve(max_size);
        	is_init = true;
		}

		bool EmplaceData(unsigned char data) {
			if(!is_init)
                Init();
			bool is_ready = false;
			if (current_byte < sync_sequence.size()) {
				message_data.clear();
				// check header
				if (data == sync_sequence.at(current_byte))
					++current_byte;
				else
					current_byte = 0;
			}
			else if (current_byte == sync_sequence.size()) {
				message_size = struct_sizes.at(static_cast<DGrX_rev_4::MID>(data)) + 1;
				if (message_size) {
					message_data.emplace_back(data);
					++current_byte;
				}
				else
					current_byte = 0;
			}
			else {
				message_data.emplace_back(data);
				if (message_data.size() == message_size) {
					is_ready = true;
					current_byte = 0;
				}
				else {
					++current_byte;
				}
			}

			return is_ready;
		}

		const std::vector<unsigned char>& GetMessageData() {
			return message_data;
		}
	} byte_sync;

	namespace rev_4 {
		int DecodePosition(DGrX_rev_4::MeasuredPositionData *message, raw_t *raw) {
			try {
				if (message == nullptr || raw == nullptr)
					throw std::runtime_error("Nullptr provided");

				auto& message_data = message->GetData();
				if (static_cast<int>(message_data.fix_quality.fix_status)) {

					overlap_counter.Update(message_data.wn);
					auto wn = message_data.wn + whole_1024_weeks + overlap_counter.Count();

					raw->time = gpst2time(static_cast<int>(wn), message_data.rcv_time * 1e-3);
					if (raw->obs.n) {
						for (std::size_t i = 0; i < used_svs.size(); ++i)
							raw->obs.data[i].time = raw->time;

						std::sort(raw->obs.data, raw->obs.data + used_svs.size(), [](const obsd_t &lhs, const obsd_t &rhs) {
							return lhs.sat < rhs.sat;
						});

						raw->ephsat = 0;
						used_svs.clear();
						return ReturnCodes::input_observation_data;
					}
					else
						return ReturnCodes::no_message;
				}
				else {
					used_svs.clear();
					return ReturnCodes::no_message;
				}
			}
			catch (...) {
				return ReturnCodes::no_message;
			}
			return ReturnCodes::no_message;
		}

		int DecodeRawData(DGrX_rev_4::RawMeasurementData *message, raw_t *raw) {
			if (used_svs.empty()) {
				raw->obs.n = 0;
			}
			if (message == nullptr)
				return ReturnCodes::error_message;

			auto &data = message->GetData();
			auto is_used = std::find(used_svs.begin(), used_svs.end(), data.PRN);
			auto cur_n = raw->obs.n;
			if (is_used == used_svs.end()) {
				used_svs.push_back(data.PRN);
				raw->obs.n++;
			}
			else
				cur_n = static_cast<int>(std::distance(used_svs.begin(), is_used));

			raw->obs.data->rcv = 0;
			raw->obs.data[cur_n].sat = data.PRN;

			raw->obs.data[cur_n].L[0] = message->L1Phase();
			raw->obs.data[cur_n].L[1] = message->L2Phase();
			raw->obs.data[cur_n].P[0] = message->L1Pseudorange() * CLIGHT;
			raw->obs.data[cur_n].P[1] = message->L2Pseudorange() * CLIGHT;
			raw->obs.data[cur_n].D[0] = static_cast<float>(message->Doppler());
			raw->obs.data[cur_n].SNR[0] = static_cast<std::uint8_t>(message->GetData().snr * 4.0);
			raw->obs.data[cur_n].LLI[0] = 0;
			raw->obs.data[cur_n].code[0] = CODE_L1C;
			if (message->GetData().PRN <= NSATGPS)
				raw->obs.data[cur_n].code[1] = CODE_L2S;
			else
				raw->obs.data[cur_n].code[1] = CODE_L2C;

			return ReturnCodes::no_message;
		}

		int DecodeEphemeris(DGrX_rev_4::GPSEphemerisData *message, raw_t *raw) {
			try {
				if (message == nullptr || raw == nullptr)
					throw std::runtime_error("Nullptr provided");

				auto &data = message->GetData();
				auto cur_sv = data.prn;
				auto &eph = raw->nav.eph[cur_sv - 1];
				raw->ephsat = cur_sv;
				eph.sat = cur_sv;
				eph.iode = data.iode;
				eph.iodc = data.iodc;
				eph.sva = data.prec_and_health.ura;
				eph.svh = data.prec_and_health.satellite_health;

				overlap_counter.Update(data.wn);
				eph.week = static_cast<int>(data.wn + whole_1024_weeks + overlap_counter.Count());

				eph.code = 1;
				eph.toe = gpst2time(eph.week, message->Toe());
				eph.toc = gpst2time(eph.week, message->Toc());
				eph.ttr = gpst2time(eph.week, message->Tow());
				eph.A = message->Roota() * message->Roota();
				eph.e = message->E();
				eph.i0 = SemicyclesToRadians(message->i0());
				eph.OMG0 = SemicyclesToRadians(message->Omega0());
				eph.omg = SemicyclesToRadians(message->Omega());
				eph.OMGd = SemicyclesToRadians(message->Omegadot());
				eph.M0 = SemicyclesToRadians(message->M0());
				eph.deln = SemicyclesToRadians(message->Deltan());
				eph.idot = SemicyclesToRadians(message->Idot());
				eph.crc = message->Crc();
				eph.crs = message->Crs();
				eph.cuc = message->Cuc();
				eph.cus = message->Cus();
				eph.cic = message->Cic();
				eph.cis = message->Cis();
				eph.toes = message->Toe();
				eph.fit = 4;
				eph.f0 = message->Af0();
				eph.f1 = message->Af1();
				eph.f2 = message->Af2();
				eph.tgd[0] = message->Tgd();

				return ReturnCodes::input_ephemeris;
			}
			catch (...) {
				return ReturnCodes::no_message;
			}

			return ReturnCodes::no_message;
		}

		int DecodeEphemeris(DGrX_rev_4::GLONASSEphemerisData *message, raw_t *raw) {
			try {
				if (message == nullptr || raw == nullptr)
					throw std::runtime_error("Nullptr provided");

				auto &data = message->GetData();
				auto cur_sv = data.sv_id - 32;
				auto &eph = raw->nav.geph[cur_sv - 1];
				eph.sat = satno(SYS_GLO, cur_sv);
				raw->ephsat = eph.sat;
				eph.frq = data.litera;
				eph.svh = data.health;
				eph.sva = data.en;
				eph.toe = utc2gpst(adjday(raw->time, message->Tb() - 10800.0));

				auto tk = data.tk.hh * 60.0*60.0 + data.tk.mm * 60.0 + data.tk.ss * 30.0; // empty field
				eph.tof = utc2gpst(adjday(raw->time, tk - 10800.0));

				eph.pos[0] = KilometersToMeters(message->X());
				eph.vel[0] = KilometersToMeters(message->Xdot());
				eph.acc[0] = KilometersToMeters(message->Xdotdot());
				eph.pos[1] = KilometersToMeters(message->Y());
				eph.vel[1] = KilometersToMeters(message->Ydot());
				eph.acc[1] = KilometersToMeters(message->Ydotdot());
				eph.pos[2] = KilometersToMeters(message->Z());
				eph.vel[2] = KilometersToMeters(message->Zdot());
				eph.acc[2] = KilometersToMeters(message->Zdotdot());

				eph.taun = message->Tn();
				eph.gamn = message->Gn();
				eph.dtaun = 0;

				return ReturnCodes::input_ephemeris;
			}
			catch (...) {
				return ReturnCodes::no_message;
			}

			return ReturnCodes::no_message;
		}

		int ConvertToRaw(DGrX_rev_4::Message *message, raw_t *raw) {
			if (message == nullptr || raw == nullptr)
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
				return DecodeEphemeris(dynamic_cast<DGrX_rev_4::GLONASSEphemerisData*>(message), raw);
				break;
			case DGrX_rev_4::MID::LLAOutputMessage:
				break;
			case DGrX_rev_4::MID::GPSEphemerisData:
				return DecodeEphemeris(dynamic_cast<DGrX_rev_4::GPSEphemerisData*>(message), raw);
				break;
			case DGrX_rev_4::MID::RAIMAlertLimit:
				break;
			case DGrX_rev_4::MID::RawMeasurementData:
				return DecodeRawData(dynamic_cast<DGrX_rev_4::RawMeasurementData*>(message), raw);
				break;
			case DGrX_rev_4::MID::ExcludedSV:
				break;
			case DGrX_rev_4::MID::FirmwareSchematicVersion:
				break;
			case DGrX_rev_4::MID::MeasuredPositionData:
				return DecodePosition(dynamic_cast<DGrX_rev_4::MeasuredPositionData*>(message), raw);
				break;
			default:
				break;
			}

			return 0;
		}
	}

#if 0
	namespace rev_9 {
		int DecodePosition(DGrX_rev_9::MeasuredPositionData *message, raw_t *raw) {
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

		int DecodeRawData(DGrX_rev_9::RawMeasurementData *message, raw_t *raw) {

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
				return DecodeRawData(dynamic_cast<DGrX_rev_9::RawMeasurementData*>(message), raw);
				break;
			case DGrX_rev_9::MID::ExcludedSV:
				break;
			case DGrX_rev_9::MID::FirmwareSchematicVersion:
				break;
			case DGrX_rev_9::MID::MeasuredPositionData:
				return DecodePosition(dynamic_cast<DGrX_rev_9::MeasuredPositionData*>(message), raw);
				break;
			default:
				break;
			}

			return 0;
		}
	}
#endif

}


#endif // !_DGRx_HPP
