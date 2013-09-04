// Simple Modbus/RS485 interface.
// Send request packet to modbus device
// Receive packet back and intrepret device reading and send sample to agend.
// 
// Copyright 2013 Neil Hancock -  
// License as GPLv3 -http://www.gnu.org/licenses/gpl.html - you may do whatever you please with this program
//
//130829: free mem 56k
//130828: Neil Hancock initial version

//Configure hardware - RS485 pin allocation
// Uses Sparkfun p/n BOB-10124   RS485 Breakout - Connect RV to IMP01 Breakout 3.3V, and GND to GND
//Pin 1 Boost - '0' default OFF, '1' enable 12V
//Pin 2 Direction Rs485 - '0' default receive , '1' Transmit - Map to RS485pin 2/RTS 
//Pin 5 U2-TX RS485 Map to BOB-10124.RS485Pin 4/Rx
//Pin 7 U2-RX RS485 Map to BOB-10124.Rs485Pin 3/Tx
//Pin8 I2C-SCL Future
//Pin9 I2C-SDA Future
pinBoost<-hardware.pin1;
pinBoost.configure(DIGITAL_OUT);//Boost
pinBoost.write(0);
pinRs485dir<-hardware.pin2;
pinRs485dir.configure(DIGITAL_OUT);//RS485Direction
pinRs485dir.write(0);
uartRs485<-hardware.uart57;

const cSleepTime_sec = 600;//Debug 10--> 5min=300 Production 900
const cUlbiFrameSize=8

const rtuRdAddrReq_blob     = "\x01\x03\x00\x00\x00\x01";//\x84\x0a";//request devices modbus address
//response (7 bytes) Addr Fn Size Num1 Num2 CrcL CrcH
const rtuRdBaudReq_blob     = "\x01\x03\x00\x01\x00\x01";//\x84\x0a";//request devices BaudRate value 
// response (7 bytes) Addr Fn Size 00 Units CrcL CrcH 
const rtuRdUnitsReq_blob    = "\x01\x03\x00\x02\x00\x01";//\x25\xca";//request devices Units of display value 
// response (7 bytes) Addr Fn Size 00 Units CrcL CrcH ( units=7 'mH20') [01 03 02 07 f9 86]
const rtuRdDecpointReq_blob = "\x01\x03\x00\x03\x00\x01";//"\x74\x0a";//request devices decimal point value
//expect Addr Fn Size 00 03 CrcL Crch  (gave me 3 ie actually in mmH20) [ 01 03 02 00 03 f8 45]
const rtuRdDataReq_blob     = "\x01\x03\x00\x04\x00\x01";//"\xc5\xcb";//request device display value 
//response  Add Fn Size Msb Lsb CrcL CrcH 
const rtuRdZeroReq_blob     = "\x01\x03\x00\x05\x00\x01";//\xc5\xcb";//request device zero value 
const rtuRdSpanReq_blob     = "\x01\x03\x00\x06\x00\x01"; //xc5\xcb";//request device span value 

modbusState<-0xf0; //Uninitialized - arbitary number to use on tracking response
dbgV<-0; //See one of dbgXxx 
const dbgMsgOut=0x01;
const dbgMsgAll=0x02;

local rxBuf_blob = blob(20); // incoming modbus msgs
 snapshotTime <-0;
 seqId_cnt<-0;
 batteryV <- 0;

ledRed_State <- 0; //Debug
ledYel_State <- 0; //Debug

/*@********************************************************
Hardware management */
function setBoostVoltOn(){pinBoost.write(1);}
function setBoostVoltOff(){pinBoost.write(0);}
function setRs485DirTx(){pinRs485dir.write(1);}//set direction out
function setRs485DirRx(){pinRs485dir.write(0);}//set direction incoming

wCRCTable <- [
0x0000, 0xC0C1, 0xC181, 0x0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040]; 

function CRC16calc(msg_blob,  msgLength){
local nTemp;
local wCRCWord = 0xFFFF;
local nData = 0;

    while (msgLength--)    {
        nTemp = 0xff & (msg_blob[nData++] ^ wCRCWord);
        wCRCWord = 0xffff& (wCRCWord >> 8);
        wCRCWord = wCRCWord ^ wCRCTable[nTemp];
   }
//    local s = "crc: "; 
//    for(local b=0; b<msgLength2; b++) {s += format(" %02x", msg_blob[b]);}  
//    s += format(" [%04x]", wCRCWord);
//    server.log(s);
   return wCRCWord;
}
/*@*********************************************************
Tx to RS485 UART and wait for items sent on wire*/
function modBusConfigure() {uartRs485.configure(9600, 8, PARITY_NONE, 1, NO_CTSRTS);}
function modBusTxSync(rtuReq_str,rtuMsgLen){
    //could be implemented with stateMc & imp.wakeup()
  
   //local txBuf_blob=blob.writestring(rtuReq_blob); - new extension after 130831 to release?? 
   local txBuf_blob = blob(rtuReq_str.len());
   foreach (c in rtuReq_str) txBuf_blob.writen(c, 'b');
   
   local wCrcWord= CRC16calc(rtuReq_str,  rtuMsgLen);   
   txBuf_blob.writen(wCrcWord,'w');
   
    if (dbgV&dbgMsgOut){
        local s = "msgTx: "; 
        for(local b=0; b<txBuf_blob.len(); b++) {s += format(" %02x", txBuf_blob[b]);}  
        server.log(s);
    }    
    modbusState = txBuf_blob[3];//State - use unique lsb of address
    setBoostVoltOn();
    imp.sleep(0.001);//Wait 1mS for realworld activation or RS485
    setRs485DirTx();
    imp.sleep(0.001);//Wait for realworld line and target recognition
    uartRs485.write(txBuf_blob);

    uartRs485.flush();//wait for the output FIFO to have all bytes sent on the wire
    imp.sleep(0.0001);//might need little wait here    
    setRs485DirRx();// make ready for receiving data and return immediately
} //end 
//function modBusTxSync(rtuReq_str){modBusTxSync(rtuReq_str,rtuReq_str.len());} //no inheritance

/*@*********************************************************
Clear RS485 UART input
Return bytes cleared */
function readRs485clear(){
    local byteCnt=0;
    local byte = uartRs485.read(); //READ IN 1st character
    while(byte != -1)    {
        byte = uartRs485.read(); //and read next (if any) character
        byteCnt++;
    }
//    if (byteCnt) server.log(format("Info: bytesCleared=%d",byteCnt));;       
    return byteCnt;
}
/*@*********************************************************
Read RS485 UART input
Return lenght of input not including CRC, or -1 if bad CRC*/
function readRs485resp(){
    local timerCnt=0; //Simple timer to prevent hanging
    local byteCnt=0;
    local byte = uartRs485.read(); 
    while((byte != -1) && (timerCnt++ <65536))   {
        rxBuf_blob.writen(byte,'b');//put read character in the buffer
        byte = uartRs485.read();    //and read the next (if any) character
        byteCnt++;
    }
    //Need to check size of expected message
   local msgLength = rxBuf_blob[2]+3;
   local wCrcWord= CRC16calc(rxBuf_blob,  msgLength);
   local crcLsb =(0xff&wCrcWord);
   local crcMsb = (0xff&(wCrcWord>>8));
   if ( (rxBuf_blob[msgLength]!=crcLsb) ||
        (rxBuf_blob[msgLength+1]!=crcMsb) ) {
       byteCnt=-1; //CRC don't match - indicate  error
   }
   //msgLength +=2; //add two crc bytes for printing
    if (0/*dbgV&dbgMsgOut*/){
        local s = "msgRx:"; for(local b=0; b<msgLength+2; b++) s += format(" %02x", rxBuf_blob[b]);  
        s += format(" [%04x]", wCrcWord);
 //       s += format(" {%02x %02x}", crcLsb,rxBuf_blob[msgLength]);
 //       s += format(" [%02x %02x]", crcMsb,rxBuf_blob[msgLength+1]);
        server.log(s);
    }
 
    // and then calculate CRC       
    return byteCnt;
}
/*@*********************************************************
When to do it - main entry */
function entryTimer(){
    imp.setpowersave(true);//For quiet ADC

    //server.log("entryTimer");
    //ledRed_State = ledRed_State?0:1;//Toogle
    //ledYel_State = ledYel_State?0:1;//Toogle
    //ledRed.write(ledYel_state);
    //ledRed.write(ledYel_State);
    modBusConfigure(); // Incase power down has changed anything
    
    snapshotTime = time();
    seqId_cnt = nv.seqId_cnt;
    if ( ++seqId_cnt >0x7fff) seqId_cnt=1; 
    nv.seqId_cnt =seqId_cnt;
    batteryV = 0;
    
    cAnlgNum <- 15;      
    for (local aLp=0; aLp< cAnlgNum;aLp++) {
        batteryV += hardware.voltage();
    }
    batteryV =  batteryV/cAnlgNum;

    //Select sensor Function to query
    dbgV=0;//dbgMsgOut;
    //modBusTxSync(rtuRdAddrReq_blob);
    //modBusTxSync(rtuRdBaudReq_blob);
    //modBusTxSync(rtuRdUnitsReq_blob);
    //modBusTxSync(rtuRdDecpointReq_blob);
     modBusTxSync(rtuRdDataReq_blob,rtuRdDataReq_blob.len());
    //modBusTxSync(rtuRdZeroReq_blob,rtuRdSpanReq_blob.len());
    //modBusTxSync(rtuRdSpanReq_blob,rtuRdSpanReq_blob.len());
    imp.sleep(0.001);//Wait 1mS for realworld delays
    readRs485clear();
    imp.wakeup(0.2,doReadRs485);//Wait for response into UART but can't sleep
}
function doReadRs485() {//chained from above
    local bytesRx=readRs485resp(); //returns # valid chars read
    local modbusSensorDepth_ft=0;
    local msgLog_bool=false;
    setBoostVoltOff();
    if (7 != bytesRx ) {
         server.log(format("Err: bytesRx=%d not enough",bytesRx));       
        local s = "Err: bytesRx=";
        s += format(" %02x", bytesRx);
        s += " insufficient. RecievedMsg:";
        for(local b=0; b<bytesRx; b++) s += format(" %02x", rxBuf_blob[b]);
        server.log(s);
    } else {
        //local msgFn = rxBuf_blob[2];
        if (dbgV&dbgMsgOut){
            local s = format("[%02x] msgRx:",modbusState); for(local b=0; b<bytesRx; b++) s += format(" %02x", rxBuf_blob[b]);  server.log(s);}
        //server.log(format("Msg  %d mm",modbusSensorDepth_ft));
        switch (modbusState) {
        case 0x0: break; //ModbusAddr
        case 0x1: break; //baud
        case 0x2: break; //Decimal point
        case 0x3: break; //Units
        case 0x4: //Measurement output
            local msd = rxBuf_blob[3] ;
            if (0x7e <msd) {
                // gone negative , so zero
                modbusSensorDepth_ft =0; 
            } else {
                modbusSensorDepth_ft =(msd<<8);
                modbusSensorDepth_ft+=rxBuf_blob[4];
            }
            msgLog_bool = true;
            server.log(format("WaterDepth %d mm",modbusSensorDepth_ft));
            break;
        case 0x5: break; //Transmitters Zero
        case 0x6: break; //Transmitters span
        default: 
        }
    }

    if (true == msgLog_bool) {
        //TODO: - processing of incoming message 
        // - log message to internal NV
        
        //Send in Thingstream.net format
        agent.send("putPsTs",{
         //   "field6": ,
         //   "field5": ,
            "field4": modbusSensorDepth_ft,
            "field3": batteryV,
            "field2": seqId_cnt,
            "field1": snapshotTime
            });/* */
    }
    //imp.sleep(0.2);
    
    //hardware.pin8.write(1);//ledRed_State);
//const cSleep_15min =(900 -(time() % 900) //wake up every 15 minutes on the 1/4 hour
    imp.onidle(function(){server.sleepfor(cSleepTime_sec-(time() % cSleepTime_sec));});
}

/*@*********************************************************
 Startup and Configure output ports and display the IMP mac address on the IMP planner
   */
local wakeReason = hardware.wakereason(); //WAKEREASON_POWER_ON WAKEREASON_TIMER WAKEREASON_SW_RESET WAKEREASON_PIN1 WAKEREASON_NEW_SQUIRREL
local wakeTime = time();
if (!("nv" in getroottable())) {
    nv <- { data = "" };
    //hourly();
}
if (!("seqId_cnt" in nv)) {
    nv.seqId_cnt <- 0;
} 
if ((false == imp.getpowersave()) ||
    (WAKEREASON_POWER_ON ==wakeReason)|| 
    (WAKEREASON_NEW_SQUIRREL==wakeReason) ) {
        imp.configure("RS485-IMP001 (ver0.0ab)", [], []);//[outputRatioMin, outputParticleMin,outputPtclSzMin,outputPtclSzMax,outputconcentration]);
        server.show(hardware.getimpeeid());
        server.log(format("impMac %s impId %s SwVer %s",imp.getmacaddress(), hardware.getimpeeid(), imp.getsoftwareversion() ));
        server.log(format("ssidMac %s FreeMem=%dK",imp.getbssid() ,imp.getmemoryfree()/1000)); 
        //imp.environment() IMP001 002 AGENT
    
    server.log(format("WakeReason %d @ time=%d seqId=%d",wakeReason, wakeTime,nv.seqId_cnt));
    server.log(format("rssi %ddBm",imp.rssi()));//Call after connection (above -67 good, down to -87 terrible)
}

entryTimer(); //go there until reboot
//EOF


