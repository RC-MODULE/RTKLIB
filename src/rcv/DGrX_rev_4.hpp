#ifndef _DGrX_REV_4_HPP
#define _DGrX_REV_4_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#pragma pack(push, 1)
class DGrX_rev_4 {
public:
	enum class MID : std::uint8_t {
		CommandAcknowledgement = 0x2B,
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

		virtual ~Message() {

		}
	protected:
		template <typename T>
		void SwapEndian(T &val) = 0;

		template <>
		void SwapEndian<std::int32_t>(std::int32_t &val) {
			std::int32_t tmp = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
			val = (tmp << 16) | ((tmp >> 16) & 0xFFFF);
		}

		template <>
		void SwapEndian<std::uint32_t>(std::uint32_t &val) {
			std::uint32_t tmp = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
			val = (tmp << 16) | (tmp >> 16);
		}

		template <>
		void SwapEndian<std::int16_t>(std::int16_t &val) {
			val = (val << 8) | ((val >> 8) & 0xFF);
		}

		template <>
		void SwapEndian<std::uint16_t>(std::uint16_t &val) {
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

	};

	class FirmwareSchematicVersion final : public Message {
	private:
		struct Data {
			std::uint8_t : 8;
			std::uint32_t fw_version = 0;
			std::uint32_t schematic_version = 0;
			std::uint32_t channel_config = 0;
			std::uint32_t board_SN = 0;
			std::uint16_t sector_0_version = 0;
		} data;
		static_assert(sizeof(DGrX_rev_4::FirmwareSchematicVersion::Data) == 19, "FirmwareSchematicVersion size is wrong");

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

		virtual MID GetMID() final {
			return MID::FirmwareSchematicVersion;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
			return data;
		}
	};

	class RawMeasurementData final : public Message {
	private:
		struct Data {
			std::uint8_t PRN = 0;
			std::uint8_t : 8;
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
				std::uint16_t used_in_solution : 1;
				std::uint16_t ephemeris_availible : 1;
				std::uint16_t l1_phase_ok : 1;
				std::uint16_t l2_phase_ok : 1;
				std::uint16_t sbas_availible : 1;
				std::uint16_t l2c_capable : 1;
				std::uint16_t glonass_m : 1;
				std::uint16_t : 8;

				void Reset() {
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
		static_assert(sizeof(DGrX_rev_4::RawMeasurementData::Data) == 37, "MeasuredPosition size is wrong");

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

		virtual MID GetMID() final {
			return MID::RawMeasurementData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
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

	class MeasuredPositionData final : public Message {
	private:
		struct Data {
			enum class RAIMState :std::uint8_t {
				ok = 0,
				raim_not_availible,
				fault_corrected,
				fault_not_corrected,
				raim_off
			};

			struct {
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
			RAIMState raim_state;
			std::uint16_t wn = 0;
		} data;
		static_assert(sizeof(DGrX_rev_4::MeasuredPositionData::Data) == 41, "MeasuredPositionData size is wrong");

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

		virtual MID GetMID() final {
			return MID::MeasuredPositionData;
		}

		void Read(std::ifstream &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
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
			return data.t_gl * std::pow(2,- 5);
		}
	};

	class GPSEphemerisData : public Message {
	private:
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
		static_assert(sizeof(DGrX_rev_4::GPSEphemerisData::Data) == 79, "GPSEphemerisData size is wrong");

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

		virtual MID GetMID() final {
			return MID::GPSEphemerisData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
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

	class GLONASSEphemerisData final : public Message {
	private:
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
		static_assert(sizeof(DGrX_rev_4::GLONASSEphemerisData::Data) == 63, "GLONASSEphemerisData size is wrong");

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

		virtual MID GetMID() final {
			return MID::GLONASSEphemerisData;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
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

	class RAIMAlertLimit final : public Message {
	private:
		struct Data {
			enum class LimitType :std::uint8_t {
				user_defined = 0,
				ert,
				trm,
				npa = 4
			};
			LimitType limit_type;
			std::uint16_t alert_limit = 0;
			std::uint16_t : 16;
		} data;
		static_assert(sizeof(DGrX_rev_4::RAIMAlertLimit::Data) == 5, "RAIMAlertLimit size is wrong");

		void Preprocess() {
			SwapEndian(data.alert_limit);
		}
	public:
		RAIMAlertLimit() = default;
		template <typename T>
		RAIMAlertLimit(T &file) {
			Read(file);
		}

		virtual MID GetMID() final {
			return MID::RAIMAlertLimit;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
			return data;
		}
	};

	class CommandAcknowledgement final : public Message {
	private:
		struct Data {
			std::uint8_t ack_id = 0;
		} data;
		static_assert(sizeof(DGrX_rev_4::CommandAcknowledgement::Data) == 1, "CommandAcknowledgement size is wrong");
	public:
		CommandAcknowledgement() = default;
		template <typename T>
		CommandAcknowledgement(T &file) {
			Read(file);
		}

		virtual MID GetMID() final {
			return MID::CommandAcknowledgement;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		auto& Data() {
			return data;
		}
	};

	class CommandNAcknowledgement final : public Message {
	private:
		struct Data {
			std::uint8_t nack_id = 0;
		} data;
		static_assert(sizeof(DGrX_rev_4::CommandNAcknowledgement::Data) == 1, "CommandNAcknowledgement size is wrong");
	public:
		CommandNAcknowledgement() = default;
		template <typename T>
		CommandNAcknowledgement(T &file) {
			Read(file);
		}

		virtual MID GetMID() final {
			return MID::CommandNAcknowledgement;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		auto& Data() {
			return data;
		}
	};

	class LLAOutputMessage final : public Message {
	private:
		struct Data {
			std::uint8_t : 8;
			std::uint32_t rcv_time = 0;
			std::int32_t lat = 0;
			std::uint32_t lon = 0;
			std::int32_t alt = 0;
		} data;
		static_assert(sizeof(DGrX_rev_4::LLAOutputMessage::Data) == 17, "LLAOutputMessage size is wrong");

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

		virtual MID GetMID() final {
			return MID::LLAOutputMessage;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
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

	class DebugData final : public Message {
	private:
		struct Data {
			std::uint8_t : 8;
			std::uint32_t clk_err_sum = 0;
			std::array<std::uint8_t, 6> clr_err_squared_sum;
			std::uint32_t vcc_err_sum = 0;
			std::array<std::uint8_t, 6> vcc_err_squared_sum;
		} data;
		static_assert(sizeof(DGrX_rev_4::DebugData::Data) == 21, "DebugData size is wrong");

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

		auto& Data() {
			return data;
		}
	};

	class ExcludedSV final : public Message {
	private:
		struct Data {
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

			std::uint8_t prn = 0;
			SystemID system_id;
			Reason reason;
		} data;
		static_assert(sizeof(DGrX_rev_4::ExcludedSV::Data) == 3, "ExcludedSV size is wrong");
	public:
		ExcludedSV() = default;
		template <typename T>
		ExcludedSV(T &file) {
			Read(file);
		}

		virtual MID GetMID() final {
			return MID::ExcludedSV;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		auto& Data() {
			return data;
		}
	};

	class AlmanacStatus final : public Message {
	private:
		struct Data {
			enum class Status : std::uint8_t {
				no_almanac = 0,
				too_old,
				updated_at_startup,
				updated_by_user_command,
				new_almanac_collected
			};

			Status status;
		} data;
		static_assert(sizeof(DGrX_rev_4::AlmanacStatus::Data) == 1, "AlmanacStatus size is wrong");
	public:
		AlmanacStatus() = default;
		template <typename T>
		AlmanacStatus(T &file) {
			Read(file);
		}

		virtual MID GetMID() final {
			return MID::AlmanacStatus;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
		}

		auto& Data() {
			return data;
		}
	};


	class ClockStatus final : public Message {
	private:
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
		static_assert(sizeof(DGrX_rev_4::ClockStatus::Data) == 23, "ClockStatus size is wrong");

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

		virtual MID GetMID() final {
			return MID::ClockStatus;
		}

		template <typename T>
		void Read(T &file) {
			file.read(reinterpret_cast<char*>(&data), sizeof(data));
			Preprocess();
		}

		auto& Data() {
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

	static std::unique_ptr<Message> ReadStruct(std::ifstream &log_file) {
		MID cur_mid;
		log_file.read(reinterpret_cast<char*>(&cur_mid), sizeof(cur_mid));

		switch (cur_mid)
		{
		case DGrX_rev_4::MID::CommandAcknowledgement:
			return std::unique_ptr<Message>(new CommandAcknowledgement(log_file));
			break;
		case DGrX_rev_4::MID::CommandNAcknowledgement:
			return std::unique_ptr<Message>(new CommandNAcknowledgement(log_file));
			break;
		case DGrX_rev_4::MID::AlmanacStatus:
			return std::unique_ptr<Message>(new AlmanacStatus(log_file));
			break;
		case DGrX_rev_4::MID::ClockStatus:
			return std::unique_ptr<Message>(new ClockStatus(log_file));
			break;
		case DGrX_rev_4::MID::GLONASSEphemerisData:
			return std::unique_ptr<Message>(new GLONASSEphemerisData(log_file));
			break;
		case DGrX_rev_4::MID::LLAOutputMessage:
			return std::unique_ptr<Message>(new LLAOutputMessage(log_file));
			break;
		case DGrX_rev_4::MID::GPSEphemerisData:
			return std::unique_ptr<Message>(new GPSEphemerisData(log_file));
			break;
		case DGrX_rev_4::MID::RAIMAlertLimit:
			return std::unique_ptr<Message>(new RAIMAlertLimit(log_file));
			break;
		case DGrX_rev_4::MID::RawMeasurementData:
			return std::unique_ptr<Message>(new RawMeasurementData(log_file));
			break;
		case DGrX_rev_4::MID::ExcludedSV:
			return std::unique_ptr<Message>(new ExcludedSV(log_file));
			break;
		case DGrX_rev_4::MID::FirmwareSchematicVersion:
			return std::unique_ptr<Message>(new FirmwareSchematicVersion(log_file));
			break;
		case DGrX_rev_4::MID::MeasuredPositionData:
			return std::unique_ptr<Message>(new MeasuredPositionData(log_file));
			break;
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
};
#pragma pack(pop)

#endif // !_DGrX_REV_4_HPP
