#include "DGrX_rev_4.hpp"
#include "DGrX_rev_9.hpp"

#ifndef WIN32
#define WIN32
#endif

#include "rtklib.h"

namespace {
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

	std::vector<std::int32_t> used_svs;

	const double Pi = 4 * std::atan(1.0);

	double SemicyclesToRadians(double val) {
		return val * Pi;
	}

	double RadiansToSemicycles(double val) {
		return val / Pi;
	}

	gtime_t adjday(gtime_t time, double tod) {
		double ep[6], tod_p;
		time2epoch(time, ep);
		tod_p = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
		if (tod<tod_p - 43200.0) tod += 86400.0;
		else if (tod>tod_p + 43200.0) tod -= 86400.0;
		ep[3] = ep[4] = ep[5] = 0.0;
		return timeadd(epoch2time(ep), tod);
	}
}

namespace rev_4 {
	int DecodePosition(DGrX_rev_4::MeasuredPositionData *message, raw_t *raw) {
		//std::array<double, 6> gpst0 = { 1980, 1, 6, 0, 0, 0 };
		//raw->time = timeadd(epoch2time(gpst0.data()), min*60.0 + msec*0.001);
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");

			auto& message_data = message->Data();
			if (static_cast<int>(message_data.fix_quality.fix_status)) {
				raw->time = gpst2time(message_data.wn + 1024, message_data.rcv_time * 1e-3);
				latest_time = raw->time;
				raw->obs.data[0].time = latest_time;
				used_svs.clear();
				if (raw->obs.n) {
					used_svs.clear();
					return ReturnCodes::input_observation_data;
				}
				else 
					return ReturnCodes::no_message;
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

#if 0
	typedef struct {        /* observation data record */
	gtime_t time;       /* receiver sampling time (GPST) */
	unsigned char sat, rcv; /* satellite/receiver number */
	unsigned char SNR[NFREQ + NEXOBS]; /* signal strength (0.25 dBHz) */
	unsigned char LLI[NFREQ + NEXOBS]; /* loss of lock indicator */
	unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
	double L[NFREQ + NEXOBS]; /* observation data carrier-phase (cycle) */
	double P[NFREQ + NEXOBS]; /* observation data pseudorange (m) */
	float  D[NFREQ + NEXOBS]; /* observation data doppler frequency (Hz) */
} obsd_t;
#endif

	int DecodeRawData(DGrX_rev_4::RawMeasurementData *message, raw_t *raw) {
		if(used_svs.empty())
			raw->obs.n = 0;
		if (std::find(used_svs.begin(), used_svs.end(), message->Data().PRN) == used_svs.end()) {
			used_svs.push_back(message->Data().PRN);
			raw->obs.n++;
		}
		auto cur_n = raw->obs.n;
		raw->obs.data[cur_n].time = latest_time;
		raw->obs.data->rcv = 0;
		raw->obs.data[cur_n].sat = message->Data().PRN;
		
		raw->obs.data[cur_n].L[0] = message->L1Phase();
		raw->obs.data[cur_n].L[1] = message->L2Phase();
		raw->obs.data[cur_n].P[0] = message->L1Pseudorange() * CLIGHT;
		raw->obs.data[cur_n].P[1] = message->L2Pseudorange() * CLIGHT;
		raw->obs.data[cur_n].D[0] = static_cast<float>(message->Doppler());
		raw->obs.data[cur_n].SNR[0] = message->Data().snr;
		raw->obs.data[cur_n].LLI[0] = 0;
		raw->obs.data[cur_n].code[0] = CODE_L1C;
		raw->obs.data[cur_n].code[1] = CODE_L2C;

		return ReturnCodes::no_message;
	}

#if 0
	typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
	int sat;            /* satellite number */
	int iode, iodc;      /* IODE,IODC */
	int sva;            /* SV accuracy (URA index) */
	int svh;            /* SV health (0:ok) */
	int week;           /* GPS/QZS: gps week, GAL: galileo week */
	int code;           /* GPS/QZS: code on L2, GAL/CMP: data sources */
	int flag;           /* GPS/QZS: L2 P data flag, CMP: nav type */
	gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
						   /* SV orbit parameters */
	double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double toes;        /* Toe (s) in week */
	double fit;         /* fit interval (h) */
	double f0, f1, f2;    /* SV clock parameters (af0,af1,af2) */
	double tgd[4];      /* group delay parameters */
						/* GPS/QZS:tgd[0]=TGD */
						/* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
						/* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
	double Adot, ndot;   /* Adot,ndot for CNAV */
} eph_t;
#endif

	int DecodeEphemeris(DGrX_rev_4::GPSEphemerisData *message, raw_t *raw) {
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");

			auto &data = message->Data();
			auto cur_sv = data.prn;
			auto &eph = raw->nav.eph[cur_sv - 1];
			raw->ephsat = cur_sv;
			eph.sat = cur_sv;
			eph.iode = data.iode;
			eph.iodc = data.iodc;
			eph.sva = data.prec_and_health.ura;
			eph.svh = data.prec_and_health.satellite_health;
			eph.week = data.wn + 1024; //
			eph.code = 1; //??
			//eph.flag = 0; //??
			eph.toe = gpst2time(eph.week, message->Toe());   ///!!! time
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

#if 0
	typedef struct {        /* GLONASS broadcast ephemeris type */
		int sat;            /* satellite number */
		int iode;           /* IODE (0-6 bit of tb field) */
		int frq;            /* satellite frequency number */
		int svh, sva, age;    /* satellite health, accuracy, age of operation */
		gtime_t toe;        /* epoch of epherides (gpst) */
		gtime_t tof;        /* message frame time (gpst) */
		double pos[3];      /* satellite position (ecef) (m) */
		double vel[3];      /* satellite velocity (ecef) (m/s) */
		double acc[3];      /* satellite acceleration (ecef) (m/s^2) */
		double taun, gamn;   /* SV clock bias (s)/relative freq bias */
		double dtaun;       /* delay between L1 and L2 (s) */
	} geph_t;
#endif

	int DecodeEphemeris(DGrX_rev_4::GLONASSEphemerisData *message, raw_t *raw) {
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");

			auto &data = message->Data();
			auto cur_sv = data.sv_id - 32;
			auto &eph = raw->nav.geph[cur_sv - 1];
			eph.sat = satno(SYS_GLO, cur_sv);
			raw->ephsat = eph.sat;
			//eph.iode = data.
			eph.frq = data.litera;
			eph.svh = data.health; ///?
			eph.sva = data.en;
			eph.toe = utc2gpst(adjday(raw->time, message->Tb() - 10800.0));

			auto tk = data.tk.hh * 60.0*60.0 + data.tk.mm * 60.0 + data.tk.ss * 30.0; //? convert to minutes?
			eph.tof = utc2gpst(adjday(raw->time, tk - 10800.0));

			eph.pos[0] = message->X();
			eph.vel[0] = message->Xdot();
			eph.acc[0] = message->Xdotdot();
			eph.pos[1] = message->Y();
			eph.vel[1] = message->Ydot();
			eph.acc[1] = message->Ydotdot();
			eph.pos[2] = message->Z();
			eph.vel[2] = message->Zdot();
			eph.acc[2] = message->Zdotdot();

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
	
	return rev_9::ConvertToRaw(DGrX_rev_9::ReadStruct(log_file).get(), raw);
}