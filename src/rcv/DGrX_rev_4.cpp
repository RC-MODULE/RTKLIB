#include "DGrX_rev_4.hpp"

#if defined(__BORLANDC__)
std::map<DGrX_rev_4::MID, std::size_t> DGrX_rev_4::struct_sizes{
	{ MID::CommandAcknowledgement,		sizeof(CommandAcknowledgement::Data) },
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
#endif
