#include "cam.h"

const byte cameraAddr = (CAM_ADDR << 5);  // addr
unsigned long picTotalLen = 0;            // picture length
File myFile;

/**
  camera functions
*/
void clearRxBuf()
{
  while (CAM_SERIAL.available())
  {
    CAM_SERIAL.read();
  }
}

void cam_sendCmd(char cmd[], int cmd_len)
{
  for (char i = 0; i < cmd_len; i++) CAM_SERIAL.print(cmd[i]);
}

void camera_initialize()
{
  char cmd[] = {0xaa, 0x0d | cameraAddr, 0x00, 0x00, 0x00, 0x00} ;
  unsigned char resp[6];
  int init_cnt = 5;
  bool init_state = false;

  CAM_SERIAL.setTimeout(500);
  while (1)
  {
    init_cnt --;
    if (0 == init_cnt) {
      break;
    }

    //clearRxBuf();
    cam_sendCmd(cmd, 6);
    if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
    {
      continue;
    }
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
    {
      if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
      if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) {
        init_state = true;
        break;
      }
    }
  }
  cmd[1] = 0x0e | cameraAddr;
  cmd[2] = 0x0d;
  cam_sendCmd(cmd, 6);
  if (init_state) {
    Serial.println("\nCamera initialization done.\n");
  } else {
    Serial.println("\nCamera initialize failed...\n");
  }
}

int preCapture(void)
{
  char cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x00, PIC_FMT };
  unsigned char resp[6];
  int err_cnt = 0;

  CAM_SERIAL.setTimeout(100);
  while (err_cnt < 10)
  {
    clearRxBuf();
    cam_sendCmd(cmd, 6);
    if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0)
    {
      return 0;
    }
    err_cnt ++;
  }

  return -1;
}

int Capture(void)
{
  char cmd[] = { 0xaa, 0x06 | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN >> 8) & 0xff , 0};
  unsigned char resp[6];
  int err_cnt = 0;

  CAM_SERIAL.setTimeout(100);
  while (err_cnt < 10)
  {
    clearRxBuf();
    cam_sendCmd(cmd, 6);
    if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
    err_cnt ++;

  }
  cmd[1] = 0x05 | cameraAddr;
  cmd[2] = 0;
  cmd[3] = 0;
  cmd[4] = 0;
  cmd[5] = 0;

  err_cnt = 0;
  while (err_cnt < 10)
  {
    clearRxBuf();
    cam_sendCmd(cmd, 6);
    if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
    err_cnt ++;

  }
  cmd[1] = 0x04 | cameraAddr;
  cmd[2] = 0x1;
  err_cnt = 0;

  while (err_cnt < 10)
  {
    clearRxBuf();
    cam_sendCmd(cmd, 6);
    if (CAM_SERIAL.readBytes((char *)resp, 6) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
    {
      CAM_SERIAL.setTimeout(1000);
      if (CAM_SERIAL.readBytes((char *)resp, 6) != 6)
      {
        continue;
      }
      if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x01)
      {
        picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
        Serial.println("picTotalLen:");
        Serial.println(picTotalLen);
        break;
      }
    }
    err_cnt ++;
  }

  return 0;
}

int GetData(void)
{
  unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
  if ((picTotalLen % (PIC_PKT_LEN - 6)) != 0) pktCnt += 1;

  char cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
  unsigned char pkt[PIC_PKT_LEN];

  char picName[] = "pic.jpg";     //make a jpg file named "pic.jpg", the file will be removed if it has already exist.

  if (SD.exists(picName)) {
    SD.remove(picName);
  }

  myFile = SD.open(picName, FILE_WRITE);
  if (!myFile) {
    Serial.print("Error: [");
    Serial.print(__LINE__);
    Serial.print("]: ");
    Serial.println("Open file error!");
    return -1;
  }

  CAM_SERIAL.setTimeout(1000);
  for (unsigned int i = 0; i < pktCnt; i++)
  {
    cmd[4] = i & 0xff;
    cmd[5] = (i >> 8) & 0xff;

    int retry_cnt = 0;
    retry:
    delay(10);
    clearRxBuf();
    cam_sendCmd(cmd, 6);
    uint16_t cnt = CAM_SERIAL.readBytes((char *)pkt, PIC_PKT_LEN);

    unsigned char sum = 0;
    for (int y = 0; y < cnt - 2; y++)
    {
      sum += pkt[y];
    }
    if (sum != pkt[cnt - 2])
    {
      if (++retry_cnt < 100) goto retry;
      else break;
    }

    myFile.write((const uint8_t *)&pkt[4], cnt - 6); //write the date getted from camera.
  }
  cmd[4] = 0xf0;
  cmd[5] = 0xf0;
  cam_sendCmd(cmd, 6);

  myFile.close();

  return 0;
}


int sendData(void) {

  const int dataSize = 127;
  byte dataBuffer[dataSize];
  int tail = 0;
  // uint32_t time_cnt = 0;
  delay(1000);
  File photoFile = SD.open("pic.jpg");

  if (!photoFile) {
    Serial.print("Error: [");
    Serial.print(__LINE__);
    Serial.print("]: ");
    Serial.println("Open photoFile error!");
    return -1;
  }

  while (photoFile.position() < photoFile.size()) {   //do when there is bytes in jpg file.

    // if(0 == (tail % dataSize)){
    //     time_cnt = millis();
    // }
    dataBuffer[tail++] = photoFile.read();   //fullfill the databuffer

    if (tail == dataSize) {        //if already get dataSize byte from jpg file
      int val;
      // Serial.print("Read photoFile by 127 bytes: ");
      // Serial.println(millis() - time_cnt);

      do {
        // time_cnt = millis();
        int sum = 0;
        for (int i = 0; i < dataSize; i++) {
          RF_SERIAL.write(dataBuffer[i]); // send the data in buffer
          sum += dataBuffer[i];
          sum = sum % 0xFF;           //calc the check byte.
        }
        RF_SERIAL.write(sum);            //send the check byte.
        tail = 0;

        // Serial.print("Send dataSize + 1 bytes: ");
        // Serial.println(millis() - time_cnt);

        while (RF_SERIAL.available() <= 0) {
        }

        val = RF_SERIAL.read();

        if (val == COMM_CAN ) {
          Serial.println("Received cancle!");
          photoFile.close();
          return -2;
        } else if (val == COMM_ACK) {
          break;
        } else if (val != COMM_NAK) { 
//          Serial.println("Received rubbish!");
          photoFile.close();
          return -3;
        }
      } while (val == COMM_NAK);
    }
  }
  if (tail > 0) {
    int val;
    do {
      int sum = 0;
      for (int i = 0; i < dataSize; i++) {
        if (i < tail) {
          RF_SERIAL.write(dataBuffer[i]);
          sum += dataBuffer[i];
          sum = sum % 0xFF;
        } else {
          RF_SERIAL.write(0);    //if there are no bytes in jpg file, fill it with 0x00;
        }
      }
      Serial.print("[");
      Serial.print(__LINE__);
      Serial.print("]: ");
      Serial.println("Sent last pic data!");
      RF_SERIAL.write(sum);
      
      while (RF_SERIAL.available() <= 0) {
      }

      val = RF_SERIAL.read();
      if (val == COMM_CAN ) {
        Serial.println("Received cancle!");
        photoFile.close();
        return -2;
      } else if (val == COMM_ACK) {
        break;
      } else if (val != COMM_NAK) { 
        Serial.println("Received rubbish!");
        photoFile.close();
        return -2;
      }
    } while (val == COMM_NAK);
  } /*End of "while (photoFile.position() < photoFile.size())"" */

  photoFile.close();

  return 0;
}


void cam_init(void) {
  // Camera init
  pinMode(5, OUTPUT);         // CS pin of SD Card Shield
  if (!SD.begin(5))
  {
    Serial.println("sd init failed\n");
    
    // return;
  } else {
    Serial.println("sd init done.\n");
  }
  CAM_SERIAL.begin(115200);
  camera_initialize();  
}





