#include "RoboClaw.h"

//
// Constructor
//
RoboClaw::RoboClaw(HardwareSerial *serial_port, uint32_t tout,bool doack) : serial_port_(serial_port)
{
	timeout = tout;
	ack=doack;
}

//
// Destructor
//
RoboClaw::~RoboClaw()
{
}

void RoboClaw::begin(long baud)
{
  serial_port_->begin(baud);
}

void RoboClaw::end()
{
  serial_port_->end();
}

int16_t RoboClaw::read(uint32_t timeout)
{
  if(!serial_port_) return -1;
  
  unsigned long start_time = millis();
  while(millis() - start_time < timeout) 
  {
    if (serial_port_->available()) 
    {
      return serial_port_->read();
    }
    yield();
  }

  return -1;
}

//Does not work on the arduino Due - need to get rid of va_args
bool RoboClaw::write_n(uint8_t cnt, ... )
{
	uint8_t crc=0;

	//send data with crc
	va_list marker;
	va_start( marker, cnt );     /* Initialize variable arguments. */
	for(uint8_t index=0;index<cnt;index++){
                uint8_t data = va_arg(marker, uint16_t);
		crc+=data;
		serial_port_->write(data);
	}
	va_end( marker );              /* Reset variable arguments.      */

	if(ack)
		serial_port_->write(crc&0x7F | 0x80);
	else
		serial_port_->write(crc&0x7F);
	if(ack)
		if(read(timeout)==0xFF)
			return true;
	return false;
}

bool RoboClaw::ForwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1FORWARD,speed);
}

bool RoboClaw::BackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1BACKWARD,speed);
}

bool RoboClaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINMB,voltage);
}

bool RoboClaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXMB,voltage);
}

bool RoboClaw::ForwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2FORWARD,speed);
}

bool RoboClaw::BackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2BACKWARD,speed);
}

bool RoboClaw::ForwardBackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M17BIT,speed);
}

bool RoboClaw::ForwardBackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M27BIT,speed);
}

bool RoboClaw::ForwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFORWARD,speed);
}

bool RoboClaw::BackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDBACKWARD,speed);
}

bool RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDRIGHT,speed);
}

bool RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLEFT,speed);
}

bool RoboClaw::ForwardBackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFB,speed);
}

bool RoboClaw::LeftRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLR,speed);
}

//Will probably not work on the Due
bool RoboClaw::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
	uint8_t crc;
	serial_port_->write(address);
	crc=address;
	serial_port_->write(cmd);
	crc+=cmd;

	//send data with crc
	va_list marker;
	va_start( marker, cnt );     /* Initialize variable arguments. */
	for(uint8_t index=0;index<cnt;index++){
		uint32_t *ptr = (uint32_t *)va_arg(marker, uint16_t);

		uint32_t value;
		uint8_t data = read(timeout);
		crc+=data;
		value=(uint32_t)data<<24;

		data = read(timeout);
		crc+=data;
		value|=(uint32_t)data<<16;

		data = read(timeout);
		crc+=data;
		value|=(uint32_t)data<<8;

		data = read(timeout);
		crc+=data;
		value|=(uint32_t)data;

		*ptr = value;
	}
	va_end( marker );              /* Reset variable arguments.      */

	uint8_t data = read(timeout);

	return ((crc&0x7F)==data);
}

uint32_t RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status,bool *valid){
	uint8_t crc;
	serial_port_->write(address);
	crc=address;
	serial_port_->write(cmd);
	crc+=cmd;

	uint32_t value;
	uint8_t data = read(timeout);
	crc+=data;
	value=(uint32_t)data<<24;

	data = read(timeout);
	crc+=data;
	value|=(uint32_t)data<<16;

	data = read(timeout);
	crc+=data;
	value|=(uint32_t)data<<8;

	data = read(timeout);
	crc+=data;
	value|=(uint32_t)data;
	
	data = read(timeout);
	crc+=data;
	if(status)
		*status = data;
		
	data = read(timeout);
	if(valid)
		*valid = ((crc&0x7F)==data);
		
	return value;
}

uint32_t RoboClaw::ReadEncM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ENC,status,valid);
}

uint32_t RoboClaw::ReadEncM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ENC,status,valid);
}

uint32_t RoboClaw::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1SPEED,status,valid);
}

uint32_t RoboClaw::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2SPEED,status,valid);
}

bool RoboClaw::ResetEncoders(uint8_t address){
	return write_n(2,address,RESETENC);
}

bool RoboClaw::ReadVersion(uint8_t address,char *version){
	uint8_t crc;
	serial_port_->write(address);
	crc=address;
	serial_port_->write(GETVERSION);
	crc+=GETVERSION;
	
	for(uint8_t i=0;i<32;i++){
		version[i]=read(timeout);
		crc+=version[i];
		if(version[i]==0){
			if((crc&0x7F)==read(timeout))
				return true;
			else
				return false;
		}
	}
	return false;
}

uint16_t RoboClaw::Read2(uint8_t address,uint8_t cmd,bool *valid){
	uint8_t crc;
	serial_port_->write(address);
	crc=address;
	serial_port_->write(cmd);
	crc+=cmd;
	
	uint16_t value;	
	uint8_t data = read(timeout);
	crc+=data;
	value=(uint16_t)data<<8;

	data = read(timeout);
	crc+=data;
	value|=(uint16_t)data;
	
	data = read(timeout);
	if(valid)
		*valid = ((crc&0x7F)==data);
		
	return value;
}

uint16_t RoboClaw::ReadMainBatteryVoltage(uint8_t address,bool *valid){
	return Read2(address,GETMBATT,valid);
}

uint16_t RoboClaw::ReadLogicBattVoltage(uint8_t address,bool *valid){
	return Read2(address,GETLBATT,valid);
}

bool RoboClaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINLB,voltage);
}

bool RoboClaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXLB,voltage);
}

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

// Hack to get this command working on the Due
bool RoboClaw::SetM1VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){
	uint32_t kd = kd_fp*65536;
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;

        uint8_t crc=0;

        serial_port_->write(address);
        crc+=address;
        serial_port_->write(SETM1PID);
        crc+=SETM1PID;
        serial_port_->write((uint8_t)(kd>>24));
        crc+=((uint8_t)(kd>>24));
        serial_port_->write((uint8_t)(kd>>16));
        crc+=((uint8_t)(kd>>16));
        serial_port_->write((uint8_t)(kd>>8));
        crc+=((uint8_t)(kd>>8));
        serial_port_->write((uint8_t)kd);
        crc+=((uint8_t)kd);
        serial_port_->write((uint8_t)(kp>>24));
        crc+=((uint8_t)(kp>>24));
        serial_port_->write((uint8_t)(kp>>16));
        crc+=((uint8_t)(kp>>16));
        serial_port_->write((uint8_t)(kp>>8));
        crc+=((uint8_t)(kp>>8));
        serial_port_->write((uint8_t)kp);
        crc+=((uint8_t)kp);
        serial_port_->write((uint8_t)(ki>>24));
        crc+=((uint8_t)(ki>>24));
        serial_port_->write((uint8_t)(ki>>16));
        crc+=((uint8_t)(ki>>16));
        serial_port_->write((uint8_t)(ki>>8));
        crc+=((uint8_t)(ki>>8));
        serial_port_->write((uint8_t)ki);
        crc+=((uint8_t)ki);
        serial_port_->write((uint8_t)(qpps>>24));
        crc+=((uint8_t)(qpps>>24));
        serial_port_->write((uint8_t)(qpps>>16));
        crc+=((uint8_t)(qpps>>16));
        serial_port_->write((uint8_t)(qpps>>8));
        crc+=((uint8_t)(qpps>>8));
        serial_port_->write((uint8_t)qpps);
        crc+=((uint8_t)qpps);
        
	if(ack){
		serial_port_->write(crc&0x7F | 0x80);
	}else{
	        serial_port_->write(crc&0x7F);
	
        }

        if(ack)
		if(read(timeout)==0xFF)
			return true;

	return false;
        
//	return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool RoboClaw::SetM2VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, uint32_t qpps){
	uint32_t kd = kd_fp*65536;
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t RoboClaw::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t RoboClaw::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ISPEED,status,valid);
}

bool RoboClaw::DutyM1(uint8_t address, uint16_t duty){
	return write_n(4,address,M1DUTY,SetWORDval(duty));
}

bool RoboClaw::DutyM2(uint8_t address, uint16_t duty){
	return write_n(4,address,M2DUTY,SetWORDval(duty));
}

bool RoboClaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2){
	return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

bool RoboClaw::SpeedM1(uint8_t address, uint32_t speed){
	return write_n(6,address,M1SPEED,SetDWORDval(speed));
}

bool RoboClaw::SpeedM2(uint8_t address, uint32_t speed){
	return write_n(6,address,M2SPEED,SetDWORDval(speed));
}

bool RoboClaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2){
	return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

bool RoboClaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}
bool RoboClaw::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2){
	return write_n(10,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClaw::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed2),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

// Hack to get this command working on the Due
bool RoboClaw::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
        uint8_t crc=0;

        serial_port_->write(address);
        crc+=address;
        serial_port_->write(M1SPEEDACCELDIST);
        crc+=M1SPEEDACCELDIST;
        serial_port_->write((uint8_t)(accel>>24));
        crc+=((uint8_t)(accel>>24));
        serial_port_->write((uint8_t)(accel>>16));
        crc+=((uint8_t)(accel>>16));
        serial_port_->write((uint8_t)(accel>>8));
        crc+=((uint8_t)(accel>>8));
        serial_port_->write((uint8_t)accel);
        crc+=((uint8_t)accel);
        serial_port_->write((uint8_t)(speed>>24));
        crc+=((uint8_t)(speed>>24));
        serial_port_->write((uint8_t)(speed>>16));
        crc+=((uint8_t)(speed>>16));
        serial_port_->write((uint8_t)(speed>>8));
        crc+=((uint8_t)(speed>>8));
        serial_port_->write((uint8_t)speed);
        crc+=((uint8_t)speed);
        serial_port_->write((uint8_t)(distance>>24));
        crc+=((uint8_t)(distance>>24));
        serial_port_->write((uint8_t)(distance>>16));
        crc+=((uint8_t)(distance>>16));
        serial_port_->write((uint8_t)(distance>>8));
        crc+=((uint8_t)(distance>>8));
        serial_port_->write((uint8_t)distance);
        crc+=((uint8_t)distance);
        serial_port_->write((uint8_t)flag);
        crc+=((uint8_t)flag);
        

	if(ack){
		serial_port_->write(crc&0x7F | 0x80);
	}else{
        	serial_port_->write(crc&0x7F);
	
        }

        if(ack)
		if(read(timeout)==0xFF)
			return true;

	return false;
  
	//return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2){
	bool valid;
	uint16_t value = Read2(address,GETBUFFERS,&valid);
	if(valid){
		depth1 = value>>8;
		depth2 = value;
	}
	return valid;
}

bool RoboClaw::ReadCurrents(uint8_t address, uint8_t &current1, uint8_t &current2){
	bool valid;
	uint16_t value = Read2(address,GETCURRENTS,&valid);
	if(valid){
		current1 = value>>8;
		current2 = value;
	}
	return valid;
}

bool RoboClaw::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
	return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

bool RoboClaw::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw::DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel){
	return write_n(6,address,M1DUTY,SetWORDval(duty),SetWORDval(accel));
}

bool RoboClaw::DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel){
	return write_n(6,address,M2DUTY,SetWORDval(duty),SetWORDval(accel));
}

bool RoboClaw::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint16_t accel1, uint16_t duty2, uint16_t accel2){
	return write_n(10,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(accel1),SetWORDval(duty2),SetWORDval(accel2));
}

bool RoboClaw::ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClaw::ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClaw::SetMainVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClaw::SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClaw::ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	uint16_t value;
	bool valid = read_n(1,address,GETMINMAXMAINVOLTAGES,&value);
	min=value>>16;
	max = value&0xFFFF;
	return valid;
}
			
bool RoboClaw::ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	uint16_t value;
	bool valid = read_n(1,address,GETMINMAXLOGICVOLTAGES,&value);
	min=value>>16;
	max = value&0xFFFF;
	return valid;
}

// Hack to get this command working on the Due
bool RoboClaw::SetM1PositionPID(uint8_t address,float kd_fp,float kp_fp,float ki_fp,float kiMax_fp,uint32_t deadzone,uint32_t min,uint32_t max){			
	uint32_t kd=kd_fp*1024;
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kiMax=kiMax_fp*1024;
        
        uint8_t crc=0;
        serial_port_->write(address);
        crc+=address;
        
        serial_port_->write(SETM1POSPID);
        crc+=SETM1POSPID;
        serial_port_->write((uint8_t)(kd>>24));
        crc+=((uint8_t)(kd>>24));
        serial_port_->write((uint8_t)(kd>>16));
        crc+=((uint8_t)(kd>>16));
        serial_port_->write((uint8_t)(kd>>8));
        crc+=((uint8_t)(kd>>8));
        serial_port_->write((uint8_t)kd);
        crc+=((uint8_t)kd);
        serial_port_->write((uint8_t)(kp>>24));
        crc+=((uint8_t)(kp>>24));
        serial_port_->write((uint8_t)(kp>>16));
        crc+=((uint8_t)(kp>>16));
        serial_port_->write((uint8_t)(kp>>8));
        crc+=((uint8_t)(kp>>8));
        serial_port_->write((uint8_t)kp);
        crc+=((uint8_t)kp);
        serial_port_->write((uint8_t)(ki>>24));
        crc+=((uint8_t)(ki>>24));
        serial_port_->write((uint8_t)(ki>>16));
        crc+=((uint8_t)(ki>>16));
        serial_port_->write((uint8_t)(ki>>8));
        crc+=((uint8_t)(ki>>8));
        serial_port_->write((uint8_t)ki);
        crc+=((uint8_t)ki);
        serial_port_->write((uint8_t)(kiMax>>24));
        crc+=((uint8_t)(kiMax>>24));
        serial_port_->write((uint8_t)(kiMax>>16));
        crc+=((uint8_t)(kiMax>>16));
        serial_port_->write((uint8_t)(kiMax>>8));
        crc+=((uint8_t)(kiMax>>8));
        serial_port_->write((uint8_t)kiMax);
        crc+=((uint8_t)kiMax);
        serial_port_->write((uint8_t)(deadzone>>24));
        crc+=((uint8_t)(deadzone>>24));
        serial_port_->write((uint8_t)(deadzone>>16));
        crc+=((uint8_t)(deadzone>>16));
        serial_port_->write((uint8_t)(deadzone>>8));
        crc+=((uint8_t)(deadzone>>8));
        serial_port_->write((uint8_t)deadzone);
        crc+=((uint8_t)deadzone);
        serial_port_->write((uint8_t)(min>>24));
        crc+=((uint8_t)(min>>24));
        serial_port_->write((uint8_t)(min>>16));
        crc+=((uint8_t)(min>>16));
        serial_port_->write((uint8_t)(min>>8));
        crc+=((uint8_t)(min>>8));
        serial_port_->write((uint8_t)min);
        crc+=((uint8_t)min);
        serial_port_->write((uint8_t)(max>>24));
        crc+=((uint8_t)(max>>24));
        serial_port_->write((uint8_t)(max>>16));
        crc+=((uint8_t)(max>>16));
        serial_port_->write((uint8_t)(max>>8));
        crc+=((uint8_t)(max>>8));
        serial_port_->write((uint8_t)max);
        crc+=((uint8_t)max);

	if(ack){
		serial_port_->write(crc&0x7F | 0x80);
	}else{
        	serial_port_->write(crc&0x7F);
	
        }

        if(ack)
		if(read(timeout)==0xFF)
			return true;

	return false;
        
	//return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw::SetM2PositionPID(uint8_t address,float kd_fp,float kp_fp,float ki_fp,float kiMax_fp,uint32_t deadzone,uint32_t min,uint32_t max){			
	uint32_t kd=kd_fp*1024;
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kiMax=kiMax_fp*1024;
	return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw::ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd,KiMax;
	bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	KiMax = ((float)KiMax_fp)/1024;
	return valid;
}

bool RoboClaw::ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd,KiMax;
	bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	KiMax = ((float)KiMax_fp)/1024;
	return valid;
}

bool RoboClaw::SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClaw::SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClaw::SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
	return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

bool RoboClaw::ReadTemp(uint8_t address, uint16_t &temp){
	bool valid;
	temp = Read2(address,GETTEMP,&valid);
	return valid;
}

uint8_t RoboClaw::ReadError(uint8_t address,bool *valid){
	uint8_t crc;
	serial_port_->write(address);
	crc=address;
	serial_port_->write(GETERROR);
	crc+=GETERROR;
	
	uint8_t value = read(timeout);
	crc+=value;

	if(valid)
		*valid = ((crc&0x7F)==read(timeout));
	else
		read(timeout);
		
	return value;
}

bool RoboClaw::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode){
	bool valid;
	uint16_t value = Read2(address,GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}

bool RoboClaw::SetM1EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM1ENCODERMODE,mode);
}

bool RoboClaw::SetM2EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM2ENCODERMODE,mode);
}

bool RoboClaw::WriteNVM(uint8_t address){
	return write_n(2,address,WRITENVM);
}
