// Copyright: (c)  2020 Andrei Ahonen
//#ifndef CANread_event_publishers_1
//#define CANread_event_publishers_1
#include "ndds/ndds_cpp.h"
#include "WorkHydraulicsRequests.h"
#include "WorkHydraulicsRequestsSupport.h"
#include "WorkHydraulicsRequestsPlugin.h"

#include "PCANBasic.h"










//#define M12_COBID_
// Values are defined in the file CANmessages.xlsx

#define M12_COBID_HYDRAULIC_REQUESTS_AND_STATUS	0x288

#define M12_COBID_WORK_PUMP_REFERENCE		0x188
#define M12_COBID_WORK_PUMP_PRESSURE		0x488
#define M12_COBID_PRESSURE_TILT_AND_LIFT 	0x189

#define M12_COBID_TILTLIFT_REF_AND_POWER	0x185
#define M12_COBID_POWER_CONSUMPTION		0x285
#define M12_COBID_FUEL_AND_OIL_INFO		0x385
#define M12_COBID_INPUT_AND_HSD_SPEED		0x485

#define M12_COBID_TILTLIFT_POSITION		0x186

#define M12_COBID_EM_STOP_CONTROLLER		0x181
#include "ndds/ndds_cpp.h"

#define M12_COBID_PUMP_TARGET_ANGLE_AND_RPM	0x18a
#define M12_COBID_PUMP_TARGET_POWER		0x28a
#define M12_COBID_PUMP_TARGET_CUTOFF		0x38a
#define M12_COBID_PUMP_ERROR			0x20a
#define M12_COBID_PUMP_R_A_PRES			0x30a
#define M12_COBID_PUMP_CURRENT_AND_TPOWER	0x40a
#define M12_COBID_PUMP_TFLOW_ROTSPEED		0x50a
#define M12_COBID_TARGET_A_P_C			0x20B

#define M12_COBID_RESOLVER			0x19f

#define M12_COBID_M4_STEER			0x214

#define M12_COBID_FUEL_CONSUMPTION		0x1e0
#define M12_COBID_FUEL_FLOW_AND_DENSITY		0x2e0

#define M12_COBID_IMU_GYRO 			0x1b2
#define M12_COBID_IMU_ACCELERATION 		0x2b2

#define M12_HEARTBEAT_RC36			0x705
#define M12_HEARTBEAT_PC104			0x707
#define M12_HEARTBEAT_DSPACE			0x708
#define M12_HEARTBEAT_EM_STOP			0x70a
#define M12_HEARTBEAT_RESOLVER			0x71f
#define M12_HEARTBEAT_M4_STEER			0x714






//#endif
