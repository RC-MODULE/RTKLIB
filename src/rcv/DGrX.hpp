#ifndef _DGRx_HPP
#define _DGRx_HPP

#include <array>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <ios>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "../rtklib.h"

#if defined(__BORLANDC__)
#include <sys/stat.h>
#endif

#ifndef WIN32
#include <sys/stat.h>
#define _fileno fileno
#define _fstat fstat
#define _stat stat
#endif

#pragma pack(push, 1)
class DataGridProtocol {
public:
	enum class MID : std::uint8_t {
		CommandAcknowledgement = 0x2B,
		L5E5G3RawMeasurement = 0x35,
		CommandNAcknowledgement = 0x3F,
		AlmanacStatus = 0x61,
		DebugData = 0x62,
		ClockStatus = 0x63,
		GLONASSEphemerisData = 0x65,
		LLAOutputMessage = 0x68,
		GPSEphemerisData = 0x69,
		RAIMAlertLimit = 0x6A,
		RawMeasurementData = 0x72,
		ExcludedSV = 0x73,
		FirmwareSchematicVersion = 0x76,
		MeasuredPositionData = 0x78
	};

	struct Message {
		virtual MID GetMID() = 0;

		virtual ~Message() = default;
	protected:
		void SwapEndian(std::int32_t &val) {
			std::int32_t tmp = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
			val = (tmp << 16) | ((tmp >> 16) & 0xFFFF);
		}

		void SwapEndian(std::uint32_t &val) {
			std::uint32_t tmp = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
			val = (tmp << 16) | (tmp >> 16);
		}

		void SwapEndian(std::int16_t &val) {
			val = (val << 8) | ((val >> 8) & 0xFF);
		}

		void SwapEndian(std::uint16_t &val) {
			val = (val << 8) | (val >> 8);
		}

		void SwapEndian(std::uint8_t *val, std::size_t size) {
			std::vector<std::uint8_t> tmp(val, val + size);
			std::reverse(tmp.begin(), tmp.end());
			for (std::size_t i = 0; i < size; ++i) {
				val[i] = tmp[i];
			}
		}

		template <typename T, std::size_t sz>
		void SwapEndian(std::array<T, sz> &arr) {
			SwapEndian(reinterpret_cast<std::uint8_t*>(arr.data()), arr.size() * sizeof(T));
		}

		bool CheckCRC(MID mid, std::uint8_t *data, std::size_t sz, std::uint16_t crc) {
			std::vector<std::uint8_t> vec(data, data + sz);
			vec.insert(vec.begin(), static_cast<std::uint8_t>(mid));

			auto ptr_16bit = reinterpret_cast<std::uint16_t*>(vec.data());

			std::uint32_t checksum = 0;
			for (std::size_t i = 0; i < sz / 2; ++i) {
				checksum += ptr_16bit[i];
			}
			return checksum == static_cast<std::uint32_t>(crc);
		}

		template <typename T, typename Struct>
		void CopyData(const std::vector<T> &data, Struct &&s) {
			assert((data.size() - 1) * sizeof(T) == sizeof(s));
			std::memcpy(&s, data.data() + 1, sizeof(s));
		}
	};

	struct FirmwareSchematicVersion final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t : 8;
			std::uint32_t fw_version = 0;
			std::uint32_t schematic_version = 0;
			std::uint32_t channel_config = 0;
			std::uint32_t board_SN = 0;
			std::uint16_t sector_0_version = 0;
		} data;
		static_assert(sizeof(DataGridProtocol::FirmwareSchematicVersion::Data) == 19, "FirmwareSchematicVersion size is wrong");

		void Preprocess() {
			SwapEndian(data.fw_version);
			SwapEndian(data.schematic_version);
			SwapEndian(data.channel_config);
			SwapEndian(data.board_SN);
			SwapEndian(data.sector_0_version);
		}
	public:
		FirmwareSchematicVersion() = default;

		template <typename T>
		FirmwareSchematicVersion(T &file) {
			Read(file);
		}

		template <typename T>
		FirmwareSchematicVersion(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::FirmwareSchematicVersion;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}
	};

	struct RawMeasurementData final : public Message {
	public:
		enum class SystemID :std::uint8_t {
			gps = 0,
			glonass,
			sbas,
			galileo_e1_e5a,
			galileo_e1_e5b,
			beidou,
			pseudolites
		};

	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t PRN = 0;
			SystemID system_id = SystemID::gps;
			std::int8_t carrier_number = 0;
			std::uint8_t L1_time_lock = 0;
			std::uint8_t L2_time_lock = 0;
			std::uint8_t sat_elevation = 0;
			std::uint8_t sat_azimuth = 0;
			std::uint8_t channel_number = 0;
			std::uint8_t snr = 0;
			std::uint8_t sig_r = 0;
			std::uint8_t sig_phi = 0;
			std::array<std::uint8_t, 6> l1_phase;
			std::uint32_t l1_pseudorange = 0;
			std::int32_t doppler = 0;
			struct {
				std::uint16_t reserved : 8;
				std::uint16_t used_in_solution : 1;
				std::uint16_t ephemeris_availible : 1;
				std::uint16_t l1_phase_ok : 1;
				std::uint16_t l2_phase_ok : 1;
				std::uint16_t sbas_availible : 1;
				std::uint16_t l2c_capable : 1;
				std::uint16_t glonass_m : 1;

				void Reset() {
					reserved = 0;
					used_in_solution = 0;
					ephemeris_availible = 0;
					l1_phase_ok = 0;
					l2_phase_ok = 0;
					sbas_availible = 0;
					l2c_capable = 0;
					glonass_m = 0;
				}
			} status;
			std::array<std::uint8_t, 6> l2_phase;
			std::uint32_t l2_pseudorange = 0;

			Data() {
				l1_phase.fill(0);
				l2_phase.fill(0);
				status.Reset();
			}
		} data;
		static_assert(sizeof(DataGridProtocol::RawMeasurementData::Data) == 37, "MeasuredPosition size is wrong");

		void Preprocess() {
			SwapEndian(data.l1_phase);
			SwapEndian(data.l1_pseudorange);
			SwapEndian(data.doppler);
			SwapEndian(data.l2_phase);
			SwapEndian(data.l2_pseudorange);
		}

	public:
		RawMeasurementData() = default;

		template <typename T>
		RawMeasurementData(T &file) {
			Read(file);
		}

		template <typename T>
		RawMeasurementData(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::RawMeasurementData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}

		double SatelliteElevation() {
			return data.sat_elevation * std::pow(2, -10);
		}

		double SatelliteAzimult() {
			return data.sat_azimuth * std::pow(2, -8);
		}

		double SigR() {
			return data.sig_r * 0.1;
		}

		double SigPhi() {
			return data.sig_phi * std::pow(2, -10);
		}

		double L1Phase() {
			std::int64_t phase_int = 0;
			for (std::size_t i = 0; i < data.l1_phase.size(); ++i)
				phase_int |= static_cast<std::int64_t>(data.l1_phase[i]) << (i * 8);

			phase_int <<= 16;
			phase_int >>= 16;
			return phase_int * std::pow(2, -12);
		}

		double L1Pseudorange() {
			return data.l1_pseudorange * std::pow(10, -10);
		}

		double L2Phase() {
			std::int64_t phase_int = 0;
			for (std::size_t i = 0; i < data.l2_phase.size(); ++i)
				phase_int |= static_cast<std::int64_t>(data.l2_phase[i]) << (i * 8);

			phase_int <<= 16;
			phase_int >>= 16;
			return phase_int * std::pow(2, -12);
		}

		double L2Pseudorange() {
			return data.l2_pseudorange * std::pow(10, -10);
		}

		double Doppler() {
			return data.doppler * std::pow(10, -4);
		}
	};

	struct  MeasuredPositionData final : public Message {
	public:
		enum class RAIMState :std::uint8_t {
			ok = 0,
			raim_not_availible,
			fault_corrected,
			fault_not_corrected,
			raim_off
		};

		enum class FixStatus : std::uint8_t {
			no_solution = 0,
			valid_fix,
			invalid_fix,
		};

		enum class LackOfSolutionReason : std::uint8_t {
			not_enough_sv = 0,
			raim_fault,
			no_convergence,
			dgps_fault,
			uncomplete_solution_fault
		};

		enum class IonosphereStatus : std::uint8_t {
			no_ionospheric_correction = 0,
			ionosphere_free_soltion
		};

		enum class SBASCorrectionStatus : std::uint8_t {
			off = 0,
			on
		};

	private:
		friend DataGridProtocol;
		struct Data {
			struct {
				std::uint8_t : 1;
				FixStatus fix_status : 2;
				LackOfSolutionReason lack_of_solution : 3;
				IonosphereStatus ionosphere_status : 1;
				SBASCorrectionStatus sbas_status : 1;
			} fix_quality;
			std::uint32_t rcv_time = 0;
			std::int32_t x_position = 0;
			std::int32_t y_position = 0;
			std::int32_t z_position = 0;
			std::int32_t r_offset = 0;
			std::int16_t x_dot = 0;
			std::int16_t y_dot = 0;
			std::int16_t z_dot = 0;
			std::int16_t r_dot = 0;
			std::int32_t t_gl = 0;
			std::uint8_t dop = 0;
			std::uint8_t gps_svs_in_fix = 0;
			std::uint8_t glonass_svs_in_fix = 0;
			std::uint8_t leap_second = 0;
			struct {
				enum class NotMixedMode :std::uint8_t {
					gps_only = 0,
					GLONASS_only,
				};
				std::uint8_t position_hold_mode : 1;
				std::uint8_t mixed_mode : 1;
				NotMixedMode not_mixed_mode : 1;
				std::uint8_t differential_solution : 1;
				std::uint8_t : 1;
				std::uint8_t antenna_present : 1;
				std::uint8_t antenna_shorted : 1;
				std::uint8_t KalmanFilterState : 1;
			} solution_mode;
			RAIMState raim_state = RAIMState::ok;
			std::uint16_t wn = 0;
		} data;
		static_assert(sizeof(DataGridProtocol::MeasuredPositionData::Data) == 41, "MeasuredPositionData size is wrong");

		void Preprocess() {
			SwapEndian(data.rcv_time);
			SwapEndian(data.x_position);
			SwapEndian(data.y_position);
			SwapEndian(data.z_position);
			SwapEndian(data.r_offset);
			SwapEndian(data.x_dot);
			SwapEndian(data.y_dot);
			SwapEndian(data.z_dot);
			SwapEndian(data.r_dot);
			SwapEndian(data.t_gl);
			SwapEndian(data.wn);
		}

	public:
		MeasuredPositionData() = default;

		template <typename T>
		MeasuredPositionData(T &file) {
			Read(file);
		}

		template <typename T>
		MeasuredPositionData(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();

		}

		virtual MID GetMID() final {
			return MID::MeasuredPositionData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}

		double XPosition() {
			return data.x_position * std::pow(2, -5);
		}

		double YPosition() {
			return data.y_position * std::pow(2, -5);
		}

		double ZPosition() {
			return data.z_position * std::pow(2, -5);
		}

		double ROffset() {
			return data.r_offset * std::pow(2, -5);
		}

		double XDot() {
			return data.x_dot * std::pow(2, -5);
		}

		double YDot() {
			return data.y_dot * std::pow(2, -5);
		}

		double ZDot() {
			return data.z_dot * std::pow(2, -5);
		}

		double RDot() {
			return data.r_offset * std::pow(2, -4);
		}

		double DOP() {
			return data.dop * std::pow(2, -3);
		}

		double Tgl() {
			return data.t_gl * std::pow(2, -5);
		}
	};

	struct GPSEphemerisData : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t prn = 0;
			std::uint32_t tow = 0;
			std::uint16_t : 16;
			std::uint16_t wn = 0;
			struct {
				std::uint16_t satellite_health : 6;
				std::uint16_t ura : 4;
				std::uint16_t : 6;

				std::uint8_t* data() {
					return reinterpret_cast<std::uint8_t*>(this);
				}
				std::size_t size() {
					return sizeof(*this);
				}
			} prec_and_health;
			std::int16_t tgd = 0;
			std::uint16_t iodc = 0;
			std::uint16_t toc = 0;
			std::int16_t af2 = 0;
			std::int16_t af1 = 0;
			std::int32_t af0 = 0;
			std::uint16_t iode = 0;
			std::int16_t cuc = 0;
			std::int16_t cus = 0;
			std::int16_t crc = 0;
			std::int16_t crs = 0;
			std::int16_t cic = 0;
			std::int16_t cis = 0;
			std::int16_t delta_n = 0;
			std::int32_t m0 = 0;
			std::uint32_t e = 0;
			std::uint32_t root_a = 0;
			std::uint16_t toe = 0;
			std::int32_t omega0 = 0;
			std::int32_t i0 = 0;
			std::int32_t omega = 0;
			std::int32_t omega_dot = 0;
			std::int16_t idot = 0;
			std::uint16_t : 16;
			std::uint32_t valid = 0;
		} data;
		static_assert(sizeof(DataGridProtocol::GPSEphemerisData::Data) == 79, "GPSEphemerisData size is wrong");

		void Preprocess() {
			SwapEndian(data.tow);
			SwapEndian(data.wn);
			SwapEndian(data.prec_and_health.data(), data.prec_and_health.size());
			SwapEndian(data.tgd);
			SwapEndian(data.iodc);
			SwapEndian(data.toc);
			SwapEndian(data.af2);
			SwapEndian(data.af1);
			SwapEndian(data.af0);
			SwapEndian(data.iode);
			SwapEndian(data.cuc);
			SwapEndian(data.cus);
			SwapEndian(data.crc);
			SwapEndian(data.crs);
			SwapEndian(data.cic);
			SwapEndian(data.cis);
			SwapEndian(data.delta_n);
			SwapEndian(data.m0);
			SwapEndian(data.e);
			SwapEndian(data.root_a);
			SwapEndian(data.toe);
			SwapEndian(data.omega0);
			SwapEndian(data.i0);
			SwapEndian(data.omega);
			SwapEndian(data.omega_dot);
			SwapEndian(data.idot);
			SwapEndian(data.valid);
		}
	
	public:
		GPSEphemerisData() = default;

		template <typename T>
		GPSEphemerisData(T &file) {
			Read(file);
		}

		template <typename T>
		GPSEphemerisData(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::GPSEphemerisData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}

		double Tgd() {
			return data.tgd * std::pow(2, -31);
		}

		double Toc() {
			return data.toc * std::pow(2, 4);
		}

		double Tow() {
			return data.tow * 6;
		}

		double Af2() {
			return data.af2 * std::pow(2, -55);
		}

		double Af1() {
			return data.af1 * std::pow(2, -43);
		}

		double Af0() {
			return data.af0 * std::pow(2, -31);
		}

		double Cuc() {
			return data.cuc * std::pow(2, -29);
		}

		double Cus() {
			return data.cus * std::pow(2, -29);
		}

		double Crc() {
			return data.crc * std::pow(2, -5);
		}

		double Crs() {
			return data.crs * std::pow(2, -5);
		}

		double Cic() {
			return data.cic * std::pow(2, -29);
		}

		double Cis() {
			return data.cis * std::pow(2, -29);
		}

		double Deltan() {
			return data.delta_n * std::pow(2, -43);
		}

		double M0() {
			return data.m0 * std::pow(2, -31);
		}

		double E() {
			return data.e * std::pow(2, -33);
		}

		double Roota() {
			return data.root_a * std::pow(2, -19);
		}

		double Toe() {
			return data.toe * std::pow(2, 4);
		}

		double Omega0() {
			return data.omega0 * std::pow(2, -31);
		}

		double i0() {
			return data.i0 * std::pow(2, -31);
		}

		double Omega() {
			return data.omega * std::pow(2, -31);
		}

		double Omegadot() {
			return data.omega_dot * std::pow(2, -43);
		}

		double Idot() {
			return data.idot * std::pow(2, -43);
		}
	};

	struct GLONASSEphemerisData final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t sv_id = 0;
			std::uint8_t : 8;
			std::int8_t litera = 0;
			std::uint16_t health = 0;
			std::uint16_t tb = 0;
			std::int32_t x = 0;
			std::int32_t y = 0;
			std::int32_t z = 0;
			std::int32_t xdot = 0;
			std::int32_t ydot = 0;
			std::int32_t zdot = 0;
			std::int16_t xdotdot = 0;
			std::int16_t ydotdot = 0;
			std::int16_t zdotdot = 0;
			struct {
				std::uint16_t ss : 1;
				std::uint16_t mm : 6;
				std::uint16_t hh : 5;
				std::uint16_t : 4;

				std::uint8_t* data() {
					return reinterpret_cast<std::uint8_t*>(this);
				}

				std::size_t size() {
					return sizeof(*this);
				}
			} tk;

			std::int32_t tn = 0;
			std::int16_t gn = 0;
			std::uint16_t en = 0;
			std::uint32_t : 32;
			std::int32_t tc = 0;
			std::uint32_t : 32;
			std::uint32_t valid = 0;

		} data;
		static_assert(sizeof(DataGridProtocol::GLONASSEphemerisData::Data) == 63, "GLONASSEphemerisData size is wrong");

		void Preprocess() {
			SwapEndian(data.health);
			SwapEndian(data.tb);
			SwapEndian(data.x);
			SwapEndian(data.y);
			SwapEndian(data.z);
			SwapEndian(data.xdot);
			SwapEndian(data.ydot);
			SwapEndian(data.zdot);
			SwapEndian(data.xdotdot);
			SwapEndian(data.ydotdot);
			SwapEndian(data.zdotdot);
			SwapEndian(data.tk.data(), data.tk.size());
			SwapEndian(data.tn);
			SwapEndian(data.gn);
			SwapEndian(data.en);
			SwapEndian(data.tc);
			SwapEndian(data.valid);
		}
	
	public:
		GLONASSEphemerisData() = default;

		template <typename T>
		GLONASSEphemerisData(T &file) {
			Read(file);
		}

		template <typename T>
		GLONASSEphemerisData(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::GLONASSEphemerisData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}

		double X() {
			return data.x * std::pow(2, -11);
		}

		double Y() {
			return data.y * std::pow(2, -11);
		}

		double Z() {
			return data.z * std::pow(2, -11);
		}

		double Xdot() {
			return data.xdot * std::pow(2, -20);
		}

		double Ydot() {
			return data.ydot * std::pow(2, -20);
		}

		double Zdot() {
			return data.zdot * std::pow(2, -20);
		}

		double Xdotdot() {
			return data.xdotdot * std::pow(2, -30);
		}

		double Ydotdot() {
			return data.ydotdot * std::pow(2, -30);
		}

		double Zdotdot() {
			return data.zdotdot * std::pow(2, -30);
		}

		double Tn() {
			return data.tn * std::pow(2, -30);
		}

		double Tb() {
			return data.tb * 15.0 * 60.0;
		}

		double Gn() {
			return data.gn * std::pow(2, -40);
		}

		double Tc() {
			return data.tc * std::pow(2, -27);
		}
	};

	struct RAIMAlertLimit final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			enum class LimitType :std::uint8_t {
				user_defined = 0,
				ert,
				trm,
				npa = 4
			};
			LimitType limit_type = LimitType::user_defined;
			std::uint16_t alert_limit = 0;
			std::uint16_t : 16;
		} data;
		static_assert(sizeof(DataGridProtocol::RAIMAlertLimit::Data) == 5, "RAIMAlertLimit size is wrong");

		void Preprocess() {
			SwapEndian(data.alert_limit);
		}
	
	public:
		RAIMAlertLimit() = default;

		template <typename T>
		RAIMAlertLimit(T &file) {
			Read(file);
		}

		template <typename T>
		RAIMAlertLimit(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::RAIMAlertLimit;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}
	};

	struct CommandAcknowledgement final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t ack_id = 0;
		} data;
		static_assert(sizeof(DataGridProtocol::CommandAcknowledgement::Data) == 1, "CommandAcknowledgement size is wrong");
	
	public:
		CommandAcknowledgement() = default;

		template <typename T>
		CommandAcknowledgement(T &file) {
			Read(file);
		}

		template <typename T>
		CommandAcknowledgement(const std::vector<T> &message) {
			CopyData(message, data);
		}

		virtual MID GetMID() final {
			return MID::CommandAcknowledgement;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		Data& GetData() {
			return data;
		}
	};

	struct CommandNAcknowledgement final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t nack_id = 0;
		} data;
		static_assert(sizeof(DataGridProtocol::CommandNAcknowledgement::Data) == 1, "CommandNAcknowledgement size is wrong");
	
	public:
		CommandNAcknowledgement() = default;

		template <typename T>
		CommandNAcknowledgement(T &file) {
			Read(file);
		}

		template <typename T>
		CommandNAcknowledgement(const std::vector<T> &message) {
			CopyData(message, data);
		}

		virtual MID GetMID() final {
			return MID::CommandNAcknowledgement;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		Data& GetData() {
			return data;
		}
	};

	struct LLAOutputMessage final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t : 8;
			std::uint32_t rcv_time = 0;
			std::int32_t lat = 0;
			std::uint32_t lon = 0;
			std::int32_t alt = 0;
		} data;
		static_assert(sizeof(DataGridProtocol::LLAOutputMessage::Data) == 17, "LLAOutputMessage size is wrong");

		void Preprocess() {
			SwapEndian(data.rcv_time);
			SwapEndian(data.lat);
			SwapEndian(data.lon);
			SwapEndian(data.alt);
		}

	public:
		LLAOutputMessage() = default;

		template <typename T>
		LLAOutputMessage(T &file) {
			Read(file);
		}

		template <typename T>
		LLAOutputMessage(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::LLAOutputMessage;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}

		double Lat() {
			return data.lat * std::pow(2, -10);
		}

		double Lon() {
			return data.lon * std::pow(2, -10);
		}

		double Alt() {
			return data.alt * std::pow(2, -5);
		}
	};

	struct DebugData final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t : 8;
			std::uint32_t clk_err_sum = 0;
			std::array<std::uint8_t, 6> clr_err_squared_sum;
			std::uint32_t vcc_err_sum = 0;
			std::array<std::uint8_t, 6> vcc_err_squared_sum;
		} data;
		static_assert(sizeof(DataGridProtocol::DebugData::Data) == 21, "DebugData size is wrong");

		void Preprocess() {
			SwapEndian(data.clk_err_sum);
			SwapEndian(data.clr_err_squared_sum);
			SwapEndian(data.vcc_err_sum);
			SwapEndian(data.vcc_err_squared_sum);
		}
	
	public:
		DebugData() = default;
		template <typename T>
		DebugData(T &file) {
			Read(file);
		}

		virtual MID GetMID() final {
			return MID::DebugData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}
	};

	struct ExcludedSV final : public Message {
	public:
		enum class SystemID : std::uint8_t {
			gps = 0,
			glonass,
			waas_egnos,
			galileo,
			beidou = 5,
			pseudolites
		};

		enum class Reason :std::uint8_t {
			unknown = 0,
			excluded_by_user,
			low_snr,
			low_elevation,
			pseudorange_error,
			raim,
			kalman_freq_check,
			kalman_pseudorange_check
		};

	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t prn = 0;
			SystemID system_id;
			Reason reason;
		} data;
		static_assert(sizeof(DataGridProtocol::ExcludedSV::Data) == 3, "ExcludedSV size is wrong");
	
	public:
		ExcludedSV() = default;

		template <typename T>
		ExcludedSV(T &file) {
			Read(file);
		}

		template <typename T>
		ExcludedSV(const std::vector<T> &message) {
			CopyData(message, data);
		}

		virtual MID GetMID() final {
			return MID::ExcludedSV;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		Data& GetData() {
			return data;
		}
	};

	struct AlmanacStatus final : public Message {
	public:
		enum class Status : std::uint8_t {
			no_almanac = 0,
			too_old,
			updated_at_startup,
			updated_by_user_command,
			new_almanac_collected
		};

	private:
		friend DataGridProtocol;
		struct Data {
			Status status;
		} data;
		static_assert(sizeof(DataGridProtocol::AlmanacStatus::Data) == 1, "AlmanacStatus size is wrong");

	public:
		AlmanacStatus() = default;

		template <typename T>
		AlmanacStatus(T &file) {
			Read(file);
		}

		template <typename T>
		AlmanacStatus(const std::vector<T> &message) {
			CopyData(message, data);
		}

		virtual MID GetMID() final {
			return MID::AlmanacStatus;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		Data& GetData() {
			return data;
		}
	};

	struct ClockStatus final : public Message {
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t : 8;
			std::uint16_t wn = 0;
			std::uint32_t rcv_time = 0;
			std::int32_t r_offset = 0;
			std::int16_t r_dot = 0;
			std::int32_t glonass_tshift = 0;
			std::int32_t glonass_tshift_almanac = 0;
			std::int16_t leap_seconds = 0;
		} data;
		static_assert(sizeof(DataGridProtocol::ClockStatus::Data) == 23, "ClockStatus size is wrong");

		void Preprocess() {
			SwapEndian(data.wn);
			SwapEndian(data.rcv_time);
			SwapEndian(data.r_offset);
			SwapEndian(data.r_dot);
			SwapEndian(data.glonass_tshift);
			SwapEndian(data.glonass_tshift_almanac);
			SwapEndian(data.leap_seconds);
		}

	public:
		ClockStatus() = default;

		template <typename T>
		ClockStatus(T &file) {
			Read(file);
		}

		template <typename T>
		ClockStatus(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::ClockStatus;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}

		double ROffset() {
			return data.r_offset * std::pow(2, -5);
		}

		double RDot() {
			return data.r_dot* std::pow(2, -4);
		}

		double GLONASS_Tshift() {
			return data.glonass_tshift * std::pow(2, -5);
		}

		double GLONASS_Tshift_Almanac() {
			return data.glonass_tshift_almanac * std::pow(2, -5);
		}
	};

	struct L5E5G3RawMeasurement final : public Message {
	public:
		enum class SignalID : std::uint8_t {
			gps_l5 = 0,
			glonass_g3,
			galileo_e5a = 3,
			galileo_e5b,
		};
	private:
		friend DataGridProtocol;
		struct Data {
			std::uint8_t sv_number = 0;
			SignalID signal_id = SignalID::gps_l5;
			std::uint8_t pll_update_cnt = 0;
			std::uint8_t snr = 0;
			std::uint8_t : 8;
			std::array<std::uint8_t, 6> l5_phase;
			std::uint32_t l5_pseudorange = 0;

			Data() {
				l5_phase.fill(0);
			}
		} data;
		static_assert(sizeof(DataGridProtocol::L5E5G3RawMeasurement::Data) == 15, "L5E5G3RawMeasurement size is wrong");

		void Preprocess() {
			SwapEndian(data.l5_phase);
			SwapEndian(data.l5_pseudorange);
		}

	public:
		L5E5G3RawMeasurement() = default;

		template <typename T>
		L5E5G3RawMeasurement(T &file) {
			Read(file);
		}

		template <typename T>
		L5E5G3RawMeasurement(const std::vector<T> &message) {
			CopyData(message, data);
			Preprocess();
		}

		virtual MID GetMID() final {
			return MID::L5E5G3RawMeasurement;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		Data& GetData() {
			return data;
		}

		double L5Phase() {
			std::int64_t phase_int = 0;
			for (std::size_t i = 0; i < data.l5_phase.size(); ++i)
				phase_int |= static_cast<std::int64_t>(data.l5_phase[i]) << (i * 8);

			phase_int <<= 16;
			phase_int >>= 16;
			return phase_int * std::pow(2, -12);
		}

		double L5Pseudorange() {
			return data.l5_pseudorange * std::pow(10, -10);
		}
	};

	std::vector<std::unique_ptr<Message>> ReadLog(const std::string &filename) {
		std::vector<std::unique_ptr<Message>> tmp_vector;

		std::ifstream log_file(filename.c_str(), std::ios::binary);
		if (!log_file.is_open())
			throw std::runtime_error("Unable to open log file " + filename);

		std::uint8_t last_byte = 0;
		while (!log_file.eof()) {
			log_file >> last_byte;
			if (Sync(log_file)) {
				tmp_vector.push_back(ReadStruct(log_file));
			}
		}

		std::vector<std::unique_ptr<Message>> dst;
		for (auto&&el : tmp_vector)
			if (el.get() != nullptr)
				dst.push_back(std::move(el));

		return dst;
	}

	template <typename T>
	static std::unique_ptr<Message> ReadStruct(const std::vector<T> &log_data) {
		MID cur_mid;
		cur_mid = static_cast<MID>(log_data.at(0));

		switch (cur_mid)
		{
		case DataGridProtocol::MID::CommandAcknowledgement:
			return std::unique_ptr<Message>(new CommandAcknowledgement(log_data));
		case DataGridProtocol::MID::L5E5G3RawMeasurement:
			return std::unique_ptr<Message>(new L5E5G3RawMeasurement(log_data));
		case DataGridProtocol::MID::CommandNAcknowledgement:
			return std::unique_ptr<Message>(new CommandNAcknowledgement(log_data));
		case DataGridProtocol::MID::AlmanacStatus:
			return std::unique_ptr<Message>(new AlmanacStatus(log_data));
		case DataGridProtocol::MID::ClockStatus:
			return std::unique_ptr<Message>(new ClockStatus(log_data));
		case DataGridProtocol::MID::GLONASSEphemerisData:
			return std::unique_ptr<Message>(new GLONASSEphemerisData(log_data));
		case DataGridProtocol::MID::LLAOutputMessage:
			return std::unique_ptr<Message>(new LLAOutputMessage(log_data));
		case DataGridProtocol::MID::GPSEphemerisData:
			return std::unique_ptr<Message>(new GPSEphemerisData(log_data));
		case DataGridProtocol::MID::RAIMAlertLimit:
			return std::unique_ptr<Message>(new RAIMAlertLimit(log_data));
		case DataGridProtocol::MID::RawMeasurementData:
			return std::unique_ptr<Message>(new RawMeasurementData(log_data));
		case DataGridProtocol::MID::ExcludedSV:
			return std::unique_ptr<Message>(new ExcludedSV(log_data));
		case DataGridProtocol::MID::FirmwareSchematicVersion:
			return std::unique_ptr<Message>(new FirmwareSchematicVersion(log_data));
		case DataGridProtocol::MID::MeasuredPositionData:
			return std::unique_ptr<Message>(new MeasuredPositionData(log_data));
		default:
			break;
		}

		return std::unique_ptr<Message>(nullptr);
	}

	template <typename T>
	static std::unique_ptr<Message> ReadStruct(T &log_file) {
		MID cur_mid;
		log_file.read(reinterpret_cast<char*>(&cur_mid), sizeof(cur_mid));

		switch (cur_mid)
		{
		case DataGridProtocol::MID::CommandAcknowledgement:
			return std::unique_ptr<Message>(new CommandAcknowledgement(log_file));
		case DataGridProtocol::MID::L5E5G3RawMeasurement:
			return std::unique_ptr<Message>(new L5E5G3RawMeasurement(log_file));
		case DataGridProtocol::MID::CommandNAcknowledgement:
			return std::unique_ptr<Message>(new CommandNAcknowledgement(log_file));
		case DataGridProtocol::MID::AlmanacStatus:
			return std::unique_ptr<Message>(new AlmanacStatus(log_file));
		case DataGridProtocol::MID::ClockStatus:
			return std::unique_ptr<Message>(new ClockStatus(log_file));
		case DataGridProtocol::MID::GLONASSEphemerisData:
			return std::unique_ptr<Message>(new GLONASSEphemerisData(log_file));
		case DataGridProtocol::MID::LLAOutputMessage:
			return std::unique_ptr<Message>(new LLAOutputMessage(log_file));
		case DataGridProtocol::MID::GPSEphemerisData:
			return std::unique_ptr<Message>(new GPSEphemerisData(log_file));
		case DataGridProtocol::MID::RAIMAlertLimit:
			return std::unique_ptr<Message>(new RAIMAlertLimit(log_file));
		case DataGridProtocol::MID::RawMeasurementData:
			return std::unique_ptr<Message>(new RawMeasurementData(log_file));
		case DataGridProtocol::MID::ExcludedSV:
			return std::unique_ptr<Message>(new ExcludedSV(log_file));
		case DataGridProtocol::MID::FirmwareSchematicVersion:
			return std::unique_ptr<Message>(new FirmwareSchematicVersion(log_file));
		case DataGridProtocol::MID::MeasuredPositionData:
			return std::unique_ptr<Message>(new MeasuredPositionData(log_file));
		default:
			break;
		}

		return std::unique_ptr<Message>(nullptr);
	}

	static bool Sync(std::ifstream &log_file) {
		std::array<char, 3> sequence_to_sync = { 'G', 'R', '8' };
		std::array<char, 3> read_sequence;
		log_file.read(read_sequence.data(), read_sequence.size());
		if (read_sequence == sequence_to_sync)
			return true;

		log_file.seekg(static_cast<std::size_t>(log_file.tellg()) - sequence_to_sync.size());
		return false;
	}

	static std::unordered_map<MID, std::size_t> GetStructSizes() {
		std::unordered_map<MID, std::size_t> struct_sizes{
			{ MID::CommandAcknowledgement,		sizeof(CommandAcknowledgement::Data) },
			{ MID::L5E5G3RawMeasurement,		sizeof(L5E5G3RawMeasurement) },
			{ MID::CommandNAcknowledgement,		sizeof(CommandNAcknowledgement::Data) },
			{ MID::AlmanacStatus,				sizeof(AlmanacStatus::Data) },
			{ MID::DebugData,					sizeof(DebugData::Data) },
			{ MID::ClockStatus,					sizeof(ClockStatus::Data) },
			{ MID::GLONASSEphemerisData,		sizeof(GLONASSEphemerisData::Data) },
			{ MID::LLAOutputMessage,			sizeof(LLAOutputMessage::Data) },
			{ MID::GPSEphemerisData,			sizeof(GPSEphemerisData::Data) },
			{ MID::RAIMAlertLimit,				sizeof(RAIMAlertLimit::Data) },
			{ MID::RawMeasurementData,			sizeof(RawMeasurementData::Data) },
			{ MID::ExcludedSV,					sizeof(ExcludedSV::Data) },
			{ MID::FirmwareSchematicVersion,	sizeof(FirmwareSchematicVersion::Data) },
			{ MID::MeasuredPositionData,		sizeof(MeasuredPositionData::Data) },
		};

		return struct_sizes;
	}
};
#pragma pack(pop)

namespace DataGridTools {
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
	std::int32_t time_lock_threshold = 5;

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
			return 0*count;
		}

		void Reset() {
			pre = post = count = 0;
		}
	} overlap_counter;

	class ByteSync final {
	private:
		std::array<unsigned char, 4> sync_sequence;
		std::vector<unsigned char> message_data;
		std::size_t current_byte = 0;
		std::size_t message_size = 0;
		bool is_init = false;
		std::unordered_map<DataGridProtocol::MID, std::size_t> struct_sizes;

	public:
		ByteSync() {
			Init();
		}

		void Init() {
			sync_sequence = { 'D', 'G', 'R', '8' };
			std::size_t max_size = 0;
			struct_sizes = DataGridProtocol::GetStructSizes();
			for (auto&el : struct_sizes)
				if (el.second > max_size)
					max_size = el.second;
			max_size += sync_sequence.size() + 1; // message: DGR8{MID}{Message data};
			message_data.reserve(max_size);
			is_init = true;
		}

		bool EmplaceData(unsigned char data) {
			if (!is_init)
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
				if(struct_sizes.find(static_cast<DataGridProtocol::MID>(data)) != struct_sizes.end())
					message_size = struct_sizes.at(static_cast<DataGridProtocol::MID>(data)) + 1;
				else {
					current_byte = 0;
					return false;
				}
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
		_fstat(_fileno(fp), &s);
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
	
	void ClearMeasurements(raw_t* raw) {
		if (used_svs.empty()) {
			for (auto sv = 0; sv < raw->obs.n; ++sv) {
				for (auto i = 0; i < 3; ++i) {
					raw->obs.data[sv].L[i] = 0;
					raw->obs.data[sv].P[i] = 0;
					raw->obs.data[sv].D[i] = 0;
					raw->obs.data[sv].SNR[i] = 0;
				}
			}
			raw->obs.n = 0;
		}
	}

	int DecodePosition(DataGridProtocol::MeasuredPositionData *message, raw_t *raw) {
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");
			
			auto& message_data = message->GetData();
			if (static_cast<int>(message_data.fix_quality.fix_status) == 1) {

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

	int DecodeRawData(DataGridProtocol::RawMeasurementData *message, raw_t *raw) {
		ClearMeasurements(raw);

		if (message == nullptr)
			return ReturnCodes::error_message;

		auto &data = message->GetData();
		auto is_used = std::find(used_svs.begin(), used_svs.end(), data.PRN);
		auto cur_n = raw->obs.n;
		if (data.snr < 30 || (data.L1_time_lock < time_lock_threshold && data.L2_time_lock < time_lock_threshold) || data.status.ephemeris_availible == 0)
			return ReturnCodes::no_message;

		if (is_used == used_svs.end()) {
			used_svs.push_back(data.PRN);
			raw->obs.n++;
		}
		else
			cur_n = static_cast<int>(std::distance(used_svs.begin(), is_used));

		auto& cur_obs_data = raw->obs.data[cur_n];

		cur_obs_data.rcv = 0;
		cur_obs_data.sat = data.PRN;

		int modifier = data.L1_time_lock > time_lock_threshold ? 1 : 0;

		cur_obs_data.L[0] = message->L1Phase() * modifier;
		cur_obs_data.P[0] = message->L1Pseudorange() * CLIGHT * modifier;
		cur_obs_data.D[0] = static_cast<float>(message->Doppler()) * modifier;
		cur_obs_data.SNR[0] = static_cast<std::uint8_t>(message->GetData().snr * 4.0) * modifier;
		cur_obs_data.LLI[0] = 0;
		cur_obs_data.code[0] = CODE_L1C;
		
		modifier = data.L2_time_lock > time_lock_threshold ? 1 : 0;

		cur_obs_data.L[1] = message->L2Phase() * modifier;
		cur_obs_data.P[1] = message->L2Pseudorange() * CLIGHT * modifier;
		if (message->GetData().PRN <= NSATGPS)
			cur_obs_data.code[1] = CODE_L2S;
		else
			cur_obs_data.code[1] = CODE_L2C;

		return ReturnCodes::no_message;
	}

	int DecodeRawData(DataGridProtocol::L5E5G3RawMeasurement *message, raw_t *raw) {
#if 1
		ClearMeasurements(raw);
		
		if (message == nullptr)
			return ReturnCodes::error_message;
		auto &data = message->GetData();

		if ((static_cast<int>(data.snr) < 30) || (static_cast<int>(data.pll_update_cnt) < time_lock_threshold))
			return ReturnCodes::no_message;

		auto is_used = std::find(used_svs.begin(), used_svs.end(), data.sv_number);
		auto cur_n = raw->obs.n;
		if (is_used == used_svs.end()) {
			used_svs.push_back(data.sv_number);
			raw->obs.n++;
		}
		else
			cur_n = static_cast<int>(std::distance(used_svs.begin(), is_used));

		raw->obs.data->rcv = 0;
		raw->obs.data[cur_n].sat = data.sv_number;
		
		raw->obs.data[cur_n].L[2] = message->L5Phase();
		raw->obs.data[cur_n].P[2] = message->L5Pseudorange() * CLIGHT;
		raw->obs.data[cur_n].SNR[2] = static_cast<std::uint8_t>(message->GetData().snr * 4);
		raw->obs.data[cur_n].code[2] = CODE_L5I;
		if (data.sv_number > NSATGPS)
			raw->obs.data[cur_n].code[2] = CODE_L3I;
#endif
		return ReturnCodes::no_message;
	}

	int DecodeEphemeris(DataGridProtocol::GPSEphemerisData *message, raw_t *raw) {
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");

			auto &data = message->GetData();
			auto cur_sv = data.prn;
			auto &eph = raw->nav.eph[cur_sv - 1];
			raw->ephsat = cur_sv;

			overlap_counter.Update(data.wn);
			auto cur_week = static_cast<int>(data.wn + whole_1024_weeks + overlap_counter.Count());
			auto cur_toe = gpst2time(cur_week, message->Toe());
			if (timediff(eph.toe, cur_toe) == 0 && eph.iode == data.iode && eph.iodc == data.iodc) 	// Same ephemeris
				return ReturnCodes::no_message;

			eph.sat = cur_sv;
			eph.iode = data.iode;
			eph.iodc = data.iodc;
			eph.sva = data.prec_and_health.ura;
			eph.svh = data.prec_and_health.satellite_health;

			eph.week = cur_week;
			eph.code = 1;
			eph.toe = cur_toe;
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

	int DecodeEphemeris(DataGridProtocol::GLONASSEphemerisData *message, raw_t *raw) {
		try {
			if (message == nullptr || raw == nullptr)
				throw std::runtime_error("Nullptr provided");

			auto &data = message->GetData();
			auto cur_sv = data.sv_id - 32;
			auto &geph = raw->nav.geph[cur_sv - 1];
			geph.sat = satno(SYS_GLO, cur_sv);

			raw->ephsat = geph.sat;
			auto calculated_toe = utc2gpst(adjday(raw->time, message->Tb() - 10800.0));

			if (timediff(geph.toe, calculated_toe) == 0.0 && data.health == geph.svh) // Save ephemeris
				return ReturnCodes::no_message;

			geph.frq = data.litera;
			geph.svh = data.health;
			geph.sva = data.en;
			geph.toe = calculated_toe;

			auto tk = data.tk.hh * 60.0*60.0 + data.tk.mm * 60.0 + data.tk.ss * 30.0; // empty field
			geph.tof = utc2gpst(adjday(raw->time, tk - 10800.0));

			geph.pos[0] = KilometersToMeters(message->X());
			geph.vel[0] = KilometersToMeters(message->Xdot());
			geph.acc[0] = KilometersToMeters(message->Xdotdot());
			geph.pos[1] = KilometersToMeters(message->Y());
			geph.vel[1] = KilometersToMeters(message->Ydot());
			geph.acc[1] = KilometersToMeters(message->Ydotdot());
			geph.pos[2] = KilometersToMeters(message->Z());
			geph.vel[2] = KilometersToMeters(message->Zdot());
			geph.acc[2] = KilometersToMeters(message->Zdotdot());

			geph.taun = message->Tn();
			geph.gamn = message->Gn();
			geph.dtaun = 0;
			
			return ReturnCodes::input_ephemeris;
		}
		catch (...) {
			return ReturnCodes::no_message;
		}

		return ReturnCodes::no_message;
	}

	int ConvertToRaw(DataGridProtocol::Message *message, raw_t *raw) {
		if (message == nullptr || raw == nullptr)
			return ReturnCodes::no_message;
		auto type = message->GetMID();
		trace(3, std::string("decode_dgr: type=" + std::to_string(static_cast<int>(type)) + "\n").c_str());

		switch (type)
		{
		case DataGridProtocol::MID::CommandAcknowledgement:
			break;
		case DataGridProtocol::MID::L5E5G3RawMeasurement:
			return DecodeRawData(static_cast<DataGridProtocol::L5E5G3RawMeasurement*>(message), raw);
		case DataGridProtocol::MID::CommandNAcknowledgement:
			break;
		case DataGridProtocol::MID::AlmanacStatus:
			break;
		case DataGridProtocol::MID::DebugData:
			break;
		case DataGridProtocol::MID::ClockStatus:
			break;
		case DataGridProtocol::MID::GLONASSEphemerisData:
			return DecodeEphemeris(static_cast<DataGridProtocol::GLONASSEphemerisData*>(message), raw);
		case DataGridProtocol::MID::LLAOutputMessage:
			break;
		case DataGridProtocol::MID::GPSEphemerisData:
			return DecodeEphemeris(static_cast<DataGridProtocol::GPSEphemerisData*>(message), raw);
		case DataGridProtocol::MID::RAIMAlertLimit:
			break;
		case DataGridProtocol::MID::RawMeasurementData:
			return DecodeRawData(static_cast<DataGridProtocol::RawMeasurementData*>(message), raw);
		case DataGridProtocol::MID::ExcludedSV:
			break;
		case DataGridProtocol::MID::FirmwareSchematicVersion:
			break;
		case DataGridProtocol::MID::MeasuredPositionData:
			return DecodePosition(static_cast<DataGridProtocol::MeasuredPositionData*>(message), raw);
		default:
			break;
		}

		return 0;
	}
}

#endif // !_DGRx_HPP
