/* Copyright 2011 David Irvine
 *
 * This file is part of Loguino
 *
 * Loguino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Loguino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Loguino.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $Rev: 153 $
 * $Author: irvined $
 * $Date: 2012-10-30 17:59:04 +0000 (Tue, 30 Oct 2012) $

*/


#include "MegaSquirt.h"
#define LAMBDA 14.1

//! Loads a new set of registers.
bool MegaSquirtData::loadData(byte newReg[112]){
	uint8_t i=0;
	for (i=0;i<=112;i++){
		reg[i]=newReg[i];
	}
	return true;
}

//! The number of seconds since the controller was initialized.
uint16_t MegaSquirtData::MegaSquirtData::seconds(){
	return ((reg[0] <<8)|reg[1]);
}


//! The pulse width for the primary injection circuit.
uint16_t MegaSquirtData::MegaSquirtData::pulseWidth1(){
	return ((reg[2] <<8)|reg[3]);
}
//! The pulse width for the secondary injection circuit.
uint16_t MegaSquirtData::MegaSquirtData::pulseWidth2(){
	return ((reg[4] <<8)|reg[5]);
}

//! The current engine RPM.
uint16_t MegaSquirtData::MegaSquirtData::rpm(){
	return ((reg[6] <<8)|reg[7]);

}

//! The ignition advance in Degrees Before Top Dead Center.
int16_t MegaSquirtData::advance(){
	return ((reg[8] <<8)|reg[9]);
}


//! A byte containing the register for the squirt status.
uint8_t MegaSquirtData::squirt(){
	return reg[10];
}

//! Bank1 Ignition firing.
bool MegaSquirtData::firing1(){
	return get_bit(squirt(), 0);
}

//! Bank2 Ignition firing.
bool MegaSquirtData::firing2(){
	return get_bit(squirt(), 1);
}

//! Injector Circuit 1 scheduled to squirt.
bool MegaSquirtData::sched1(){
	return get_bit(squirt(), 2);
}

//! Injector Circuit 1 Squirting.
bool MegaSquirtData::inj1(){
	return get_bit(squirt(), 3);
}

//! Injector Circuit 2 scheduled to squirt.
bool MegaSquirtData::sched2(){
	return get_bit(squirt(), 4);
}

//! Injector Circuit 2 Squirting.
bool MegaSquirtData::inj2(){
	return get_bit(squirt(), 5);
}



//! Returns the register containing the Engine Operating/Status variables.
uint8_t MegaSquirtData::engine(){
	return reg[11];
}

//! Engine ready to run.
bool MegaSquirtData::ready(){
	return get_bit(engine(),0);
}

//! Engine Cranking.
bool MegaSquirtData::crank(){
	return get_bit(engine(),1);
}

//! Engine in After Start Warmup Enrichment.
bool MegaSquirtData::startw(){
	return get_bit(engine(),2);
}

//! Engine in Warmup Enrichment.
bool MegaSquirtData::warmup(){
	return get_bit(engine(), 3);
}

//! TPS Based Acceleration active.
bool MegaSquirtData::tpsaen(){
	return get_bit(engine(), 4);
}

//! TPS Based Deceleration active.
bool MegaSquirtData::tpsden(){
	return get_bit(engine(), 5);
}


//! MAP Based Acceleration enrichment active.
bool MegaSquirtData::mapaen(){
	return get_bit(engine(), 6);
}


//! AFR target Table 1.
uint8_t MegaSquirtData::afrtgt1(){
	return reg[12];
}

//! AFR target Table 2.
uint8_t MegaSquirtData::afrtgt2(){
	return reg[13];
}

//! Wideband O2 Sensor1 enabled.  From wbo2 - indicates whether wb afr valid.
uint8_t MegaSquirtData::wbo2_en1(){
	return reg[14];
}

//! Wideband O2 Sensor2enabled.  From wbo2 - indicates whether wb afr valid.
uint8_t MegaSquirtData::wbo2_en2(){
	return reg[15];
}



//! Barometric pressure in kPa x 10.   
int16_t MegaSquirtData::barometer(){
	return ((reg[16] <<8)|reg[17]);
}

//! Manifold Air Pressure in kPA x 10.
int16_t MegaSquirtData::map(){
	return ((reg[18] <<8)|reg[19]);
}

//! Manifold Air Temperature in Fahrenheit x 10.
int16_t MegaSquirtData::mat(){
	return ((reg[20] <<8)|reg[21]);
}

//! Coolant Temperature in Fahrenheit x 10.
int16_t MegaSquirtData::coolant(){
	return int16_t(((reg[22] <<8)|reg[23]));
}

//! Throttle Position.
int16_t MegaSquirtData::tps(){
	return ((reg[24] <<8)|reg[25]);
}

//! Battery Voltage.
int16_t MegaSquirtData::batteryVoltage(){
	return ((reg[26] <<8)|reg[27]);
}

//! AFR Sensor 1.
int16_t MegaSquirtData::afr1(){
	return ((reg[28] <<8)|reg[29]);
}

//! AFR Sensor 2.
int16_t MegaSquirtData::afr2(){
	return ((reg[30] <<8)|reg[31]);

}

//! Knock Sensor. 
int16_t MegaSquirtData::knock(){
	return ((reg[32] <<8)|reg[33]);
}

//! AFR 1 as Lambda
int16_t MegaSquirtData::lambda1(){
	return afr1()/LAMBDA;
}

//! AFR 2 as Lambda
int16_t MegaSquirtData::lambda2(){
	return afr2()/LAMBDA;
}

//! Percent adjustment due to EGO Correction.
int16_t MegaSquirtData::egoCorrection1(){
	return ((reg[34] <<8)|reg[35]);
}


//! Percent adjustment due to EGO Correction.
int16_t MegaSquirtData::egoCorrection(){
	return ((egoCorrection1()+egoCorrection2())/2);
}


//! Percent adjustment due to EGO Correction.
int16_t MegaSquirtData::egoCorrection2(){
	return ((reg[36] <<8)|reg[37]);
}

//! Percent adjustment due to air temperature.
int16_t MegaSquirtData::airCorrection(){
	return ((reg[38] <<8)|reg[39]);
}

int16_t MegaSquirtData::warmupEnrich(){
	return ((reg[40] <<8)|reg[41]);

}


int16_t MegaSquirtData::accelEnrich(){
	return ((reg[42] <<8)|reg[43]);

}


int16_t MegaSquirtData::tpsfuelcut(){
	return ((reg[44] <<8)|reg[45]);

}


int16_t MegaSquirtData::baroCorrection(){
	return ((reg[46] <<8)|reg[47]);

}


int16_t MegaSquirtData::gammaEnrich(){
	return ((reg[48] <<8)|reg[49]);

}


int16_t MegaSquirtData::veCurr1(){
	return ((reg[50] <<8)|reg[51]);

}


int16_t MegaSquirtData::veCurr2(){
	return ((reg[52] <<8)|reg[53]);

}


int16_t MegaSquirtData::veCurr(){
	return veCurr1();
}


int16_t MegaSquirtData::iacstep(){
	return ((reg[54] <<8)|reg[55]);
}


int16_t MegaSquirtData::coldAdvDeg(){
	return ((reg[56] <<8)|reg[57]);
}


int16_t MegaSquirtData::tpsDOT(){
	return ((reg[58] <<8)|reg[59]);
}


int16_t MegaSquirtData::mapDOT(){
	return ((reg[60] <<8)|reg[61]);
}


int16_t MegaSquirtData::dwell(){
	return ((reg[62] <<8)|reg[63]);
}


int16_t MegaSquirtData::maf(){
	return ((reg[64] <<8)|reg[65]);
}


int16_t MegaSquirtData::calcMAP(){
	return ((reg[66] <<8)|reg[67]);

}


int16_t MegaSquirtData::fuelCorrection(){
	return ((reg[68] <<8)|reg[69]);

}

uint16_t MegaSquirtData::MegaSquirtData::portStatus()
{
	return reg[70];
}
bool MegaSquirtData::port0()
{
	return get_bit(portStatus(),0);
}
bool MegaSquirtData::port1()
{

	return get_bit(portStatus(),2);
}
bool MegaSquirtData::port2()
{
	return get_bit(portStatus(),2);

}
bool MegaSquirtData::port3()
{
	return get_bit(portStatus(),3);

}
bool MegaSquirtData::port4()
{
	return get_bit(portStatus(),4);

}
bool MegaSquirtData::port5()
{
	return get_bit(portStatus(),5);

}
bool MegaSquirtData::port6()
{
	return get_bit(portStatus(),6);

}

uint8_t MegaSquirtData::knockRetard()
{
	return reg[71];
}

int16_t MegaSquirtData::xTauFuelCorr1()
{
	return ((reg[72] <<8)|reg[73]);

}
int16_t MegaSquirtData::egoV1()
{
	return ((reg[74] <<8)|reg[75]);

}
int16_t MegaSquirtData::egoV2()
{
	return ((reg[76] <<8)|reg[77]);

}
int16_t MegaSquirtData::amcUpdates()
{
	return ((reg[78] <<8)|reg[79]);

}


int16_t MegaSquirtData::kpaix()
{
	return ((reg[80] <<8)|reg[81]);
}

//! X TAU Adjustment for VE table 2.
int16_t MegaSquirtData::xTauFuelCorr2()
{
	return ((reg[82] <<8)|reg[83]);
}






int16_t MegaSquirtData::spare1()
{
	return ((reg[84] <<8)|reg[85]);

}
int16_t MegaSquirtData::spare2()
{
	return ((reg[86] <<8)|reg[87]);

}
int16_t MegaSquirtData::trig_fix()
{
	return ((reg[88] <<8)|reg[89]);

}
int16_t MegaSquirtData::spare4()
{
	return ((reg[90] <<8)|reg[91]);

}
int16_t MegaSquirtData::spare5()
{
	return ((reg[92] <<8)|reg[93]);

}
int16_t MegaSquirtData::spare6()
{
	return ((reg[94] <<8)|reg[95]);

}
int16_t MegaSquirtData::spare7()
{
	return ((reg[96] <<8)|reg[97]);

}
int16_t MegaSquirtData::spare8()
{
	return ((reg[98] <<8)|reg[99]);

}
int16_t MegaSquirtData::spare9()
{
	return ((reg[100] <<8)|reg[101]);

}

//! Spare Byte 10.
int16_t MegaSquirtData::spare10()
{
	return ((reg[102] <<8)|reg[103]);

}

//! Incremented on each tach pulse received.
uint16_t MegaSquirtData::tachCount()
{
	return ((reg[104] <<8)|reg[105]);
}


uint8_t MegaSquirtData::ospare()
{
	return reg[106];
}

uint8_t MegaSquirtData::cksum()
{
	return reg[107];
}



//! Normalized time between trigger pulses, "fills in" missing teeth.
uint32_t MegaSquirtData::deltaT()
{
	return ( (reg[108]<<24) | (reg[109]<<16) | (reg[110]<<8) | (reg[111]) );
}






bool MegaSquirtData::get_bit(byte b, int p)
{
    b<<=(7-p);
	if (b>=127){
		return true;
	}
	return false;	
}




byte MegaSquirt::begin(){
	MS_PORT.begin(115200);
}
//! Sends the specified array of commands to the megasquirt and sets the 
//! contents of data to the data returned by the controller.  If 
//! the controller returns the exact amount of data specified, then
//! returns MS_COMM_SUCCESS.  
//! 
//! However, if the amount of data returned by the controller is less than
//! the amount expected, returns MS_ERR_COMM_TIMEOUT.  If the amount
//! of data returned is more than the amount expected, returns 
//! MS_ERR_COMM_OVERFLOW
//! 
//! Use this to send an arbitrary command to the MegaSquirt, and capture
//! any bytes returned by the controller.
//!
//! @param cmd An array of bytes to send to the controller
//! @param cmdLength The length of the cmd array.
//! @param data An array of bytes that will be populated with the data returned from the MegaSquirt
//! @param dataLength The number of bytes to be returned by the ECU.

byte MegaSquirt::runCommand(byte cmd[], byte cmdLength,  byte data[], byte dataLength)
{	
	// Flush any leftover data from the last command.
	MS_PORT.flush();
	
	// Send the specified command to the controller.
	byte i;
	for (i=0;i<cmdLength;i++){
		MS_PORT.write(cmd[i]);
	}
    
	unsigned long timeOut;
    int counter;
	
	// The Serial buffer is 64 bytes, this is less than the largest payload
	// from the controller, as such don't wait, start reading the data right
	// away and don't stop until either the requested number of bytes has 
	// been read or the timeout is reached.
	//
	counter=0;
	timeOut=millis()+MS_WAIT_TIME;
	while (counter<dataLength && millis()<timeOut)
    {
        while(counter<dataLength && MS_PORT.available()){
            data[counter]=MS_PORT.read();
            ++counter;
        }
    }
	// If there is still data pending to be read, raise OVERFLOW error.
	if (MS_PORT.available()>0 && counter>=dataLength)
	{
		return MS_ERR_COMM_OVERFLOW;
	}
    
    // If the wrong amount of data was read, return timeout.
    if (counter != dataLength){
        return MS_ERR_COMM_TIMEOUT;
    }
	// Otherwise return success.
	return MS_COMM_SUCCESS;
}


//! Sets sig to the signature of the megasquirt controller.
//! Use this to check the signature and version of the megasquirt.
//! @param sig Pointer to a String that will be set to the signature. 
byte MegaSquirt::signature(String *sig)
{

	// Signature (from Megasquirt main.c) is 32 bytes long.
	byte data[32];
	byte status;
	byte cmd[1];
	cmd[0]='S';
	status=runCommand(cmd,1,data,32);
	byte i=0;
	for (i=0;i<32;i++){
		*sig=*sig+(char)data[i];
	}
	return status;
}


//! Sets secs to the number of seconds the controller has been online.
//! The megasquirt controller records its uptime in seconds, this is primarily used for testing
//! communications with the controller.
//! @param secs pointer to  uint16_t, that gets set to the number of seconds since power on.
byte MegaSquirt::seconds(uint16_t *secs)
{
	byte status;
	byte data[2];
	byte cmd[1];
	cmd[0]='c';
	status=runCommand(cmd,1,data,2);
	// combine the two bytes into one 16 bit value.
	*secs=((uint16_t)data[0]<<8)|data[1];
	return status;
}


//! Sets rev to the revision string returned by the megasquirt controller.
//! The revision string contains the version of the megasquirt controller.
//! @param rev Pointer to a string containing the revision of the controller.
byte MegaSquirt::revision(String *rev)
{
	byte data[20];
	byte status;
	byte cmd[1];
	cmd[0]='Q';
	status=runCommand(cmd,1,data,20);
	byte i=0;
	for (i=0;i<20;i++){
		*rev=*rev+String((char)data[i]);
	}

	return status;
}


//! Queries the megasquirt for a dump of all registers
//! @param table An array of MS_TABLE_SIZE bytes which is populated with the register data
/*

	Offset	Size	Description
	0		16		seconds
	2		16		pulsewidth1
	4		16		pulswidth 2
	6		16		rpm
	8		16		advance
	10		8		engine
	11		8		megasquirt
	12		8		AFR Target 1
	13		8		AFR Target 2
	14		8		wb02 1
	15		8		wb02 2
	16		16		barometer
	18		16		map
	20		16		mat
	22		16		coolant
	24		16		tps
	26		16		battery voltage
	28		16		afr1
	30		16		afr2
	32		16		knock
	34		16		egoCorrection1
	36		16		EgoCorrection2
	38		16		airCorrection
	40		16		Warmup Enrichment
	42		16		accelEnrich
	44		16		tpsfuelcut
	46		16		baroCorrection
	48		16		gammaEnrich
    50		16		veCurr1
    52		16		veCurr2
   54		16		iacstep 
   56		16		idleDC   
   56		16		coldAdvDeg
   58		16		tpsDOT   
   60		16		mapDOT    
   62		16		dwell      
   64		16		maf         
   66		16		calcMAP      
   68		16		fuelCorrection
   70		8		portStatus
   71		8		knockRetard      
   72		16		xTauFuelCorr1   
   74		16		egoV1          
   76		16		egoV2         
   78		16		amcUpdates   
   80		16		kpaix       
   82		16		xTauFuelCorr2
   84		16		spare1      
   86		16		spare2     
   88		16		trig_fix  
   90		16		spare4   
   92		16		spare5  
   94		16		spare6 
   96		16		spare7
   98		16		spare8
   100		16		spare9
   102		16		spare10
   104		16		tachCount
   106		8		ospare 
   107		8		cksum 
   108		32		deltaT
*/
byte MegaSquirt::getData(byte table[])
{
	byte status;
	byte cmd[3];
	cmd[0]='a';
	cmd[1]=byte(0);
	cmd[2]=byte(6);
    Serial.println("Getting data from command a");
	status=runCommand(cmd,3, table,MS_TABLE_LENGTH);
	return status;
}




