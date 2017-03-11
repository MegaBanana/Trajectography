import g4p_controls.*;
import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import papaya.*;
import java.awt.*;

Serial serialPort;

float deltaT   =  0.1; // secondes

// Filtering
float DATA_FILTER_MIN_EDGE = 0.0f;
float DATA_FILTER_MAX_EDGE = 1000.0f;
float kFilteringFactor = 0.33;

float posX = 0.0;
float posY = 0.0;
float posZ = 0.0;

float oldSpeedX = 0.0;
float oldSpeedY = 0.0;
float oldSpeedZ = 0.0;

float[] accel , accelRotated,filteredAccel;
float[][] shiftReg;

float pitch;
float yaw;
float roll;

// Transform                       
float gravity[] = {0.0,0.0,0.0};

boolean DEBUG = true;
int ARRAY_SIZE = 5;

// 3D Reprensentation of the IMU
ViewObject3D viewObject3D;
SecondApplet plot;
PrintWriter output;
void setup() {
  surface.setVisible(false); //<>// //<>// //<>// //<>// //<>//

 // *********** Vue object 3D ************* 
  String[] args = {"3D View"};
  viewObject3D = new ViewObject3D();
  PApplet.runSketch(args, viewObject3D);
  
  // ********** Plot window *************** //<>// //<>// //<>//
  String[] plotName = {"Plot"};
  plot = new SecondApplet();
  PApplet.runSketch(plotName,plot);
   
  // ********** Serial data ***************
  //pitch = new SensorData(ARRAY_SIZE);
  //roll  = new SensorData(ARRAY_SIZE);
  //yaw   = new SensorData(ARRAY_SIZE);
  
  serialPort = new Serial(this, Serial.list()[1],115200);
  serialPort.bufferUntil('\n');
  
  //********************************
  output = createWriter("accel2.txt"); 
  
  accel         = new float[3];
  accelRotated  = new float[3];
  filteredAccel  = new float[3];
  shiftReg = new float[3][3];
}


void serialEvent(Serial p) {

  String part = p.readStringUntil('\n');
  if(part.length() == 36 ) {
    String array[] = part.split("\t\t"); //<>// //<>// //<>// //<>//
    if(array.length == 6) { //<>// //<>//
    //printFile(array);
    // Write the coordinate to the file
    accel[0] = ByteBuffer.wrap(array[0].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat();//lowPassFilter(ByteBuffer.wrap(array[0].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat(),0.4,accel[0]); //<>// //<>// //<>//
    accel[1] = ByteBuffer.wrap(array[1].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat();//lowPassFilter(ByteBuffer.wrap(array[1].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat(),0.4,accel[1]); //<>// //<>// //<>//
    accel[2] = ByteBuffer.wrap(array[2].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat();
  
    pitch  = ByteBuffer.wrap(array[3].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat();//lowPassFilter(ByteBuffer.wrap(array[3].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat(),0.4,pitch);
    yaw    = ByteBuffer.wrap(array[4].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat();//lowPassFilter(ByteBuffer.wrap(array[4].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat(),0.4,yaw); 
    roll   = ByteBuffer.wrap(array[5].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat();//lowPassFilter(ByteBuffer.wrap(array[5].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat(),0.4,roll);

    float c1 = cos(yaw);
    float s1 =  sin(yaw);
    
    float c2 = cos(pitch) ;
    float s2 = sin(pitch) ;
    
    float c3 = cos(roll);
    float s3 = sin(roll);
   
    
    // YAW -> PITCH -> ROLL <=> Z -> Y -> X
      float[][] rotation  = new float[][]  {{ c1 * c2   ,  c1 * s2 * s3 - c3 * s1  ,  s1* s3 + c1* c3 * s2  },
                                          { c2 * s1     ,  c1 * c3 + s1 * s2 * s3  , c3 * s1 * s2 - c1 * s3     },
                                          { -s2         ,  c2 * s3                 ,  c2 * c3  }};


   shiftReg[0][2] = shiftReg[0][1];
   shiftReg[1][2] = shiftReg[0][1];
   shiftReg[2][2] = shiftReg[0][1];
   
   shiftReg[0][1] = shiftReg[0][0];
   shiftReg[1][1] = shiftReg[0][0];
   shiftReg[2][1] = shiftReg[0][0];
   
   shiftReg[0][0] = accel[0] * rotation[0][0] + accel[1] * rotation[0][1] + accel[2] * rotation[0][2];   
   shiftReg[1][0] = accel[0] * rotation[1][0] + accel[1] * rotation[1][1] + accel[2] * rotation[1][2];   
   shiftReg[2][0] = accel[0] * rotation[2][0] + accel[1] * rotation[2][1] + accel[2] * rotation[2][2] - 1;   
   
   filteredAccel[0] = (-0.328581437337850946) * shiftReg[0][0] + (0.657162874675701891) * shiftReg[0][1] + (-0.328581437337850946) * shiftReg[0][2];
   filteredAccel[1] = (-0.328581437337850946) * shiftReg[1][0] + (0.657162874675701891) * shiftReg[1][1] + (-0.328581437337850946) * shiftReg[1][2];
   filteredAccel[2] = (-0.328581437337850946) * shiftReg[2][0] + (0.657162874675701891) * shiftReg[2][1] + (-0.328581437337850946) * shiftReg[2][2];
   
   
    // // Rotation matrix
    // //gravity[0] =  -sin(pitch);
    // //gravity[1] =   cos(pitch) * sin(roll);
    // //gravity[2] =   cos(pitch) * cos(roll);
    // 
    // //pitch yaw roll
    // float   r00,r01,r02,
    //         r10,r11,r12,
    //         r20,r21,r22;
    // 
    // 
    // r00 =  cos(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch);
    // r01 = -sin(yaw) * cos(roll);
    // r02 =  cos(yaw) * sin(pitch) + sin(yaw) * sin(roll) * cos(pitch);
    // r10 =  sin(yaw) * cos(pitch) - sin(yaw) * sin(roll) * sin(pitch);
    // r11 =  cos(yaw) * cos(roll);
    // r12 =  sin(yaw) * sin(pitch) - cos(yaw) * sin(roll) * cos(pitch);
    // r20 = -cos(roll) * sin(pitch);
    // r21 =  sin(roll);
    // r22 =  cos(roll) * cos(pitch);
   


    // gravity[0] = -cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);
    // gravity[1] =   cos(roll) * sin(pitch) * sin(yaw) + sin(roll) * cos(yaw);
    // gravity[2] =   cos(pitch) * cos(roll);
    }
  }

}

void printFile(String array[])
{
    
     output.println(
       ByteBuffer.wrap(array[0].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat()
       + " "
       + ByteBuffer.wrap(array[1].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat()
       + " " + 
       ByteBuffer.wrap(array[2].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat()
       //+ " " +
       //ByteBuffer.wrap(array[3].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat()
       //+ " " +
       //ByteBuffer.wrap(array[4].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat()
       //+ " " +
       //ByteBuffer.wrap(array[5].getBytes()).order(ByteOrder.LITTLE_ENDIAN).getFloat()
     );
}

// **** Send data to views *****
  void draw() {
    
    // Set angles
    setPitch(pitch);
    setYaw(yaw);
    setRoll(roll);

    // Remove gravity
   
    //oldSpeedX = getNewSpeed(oldSpeedX,accel[0]);
    //oldSpeedY = getNewSpeed(oldSpeedY,accel[1]);
    //oldSpeedZ = getNewSpeed(oldSpeedZ,accel[2]);
    
    //posX = getNewPos(posX,oldSpeedX);
    //posY = getNewPos(posY,oldSpeedY);
    //posZ = getNewPos(posZ,oldSpeedZ);
    
    plot.setPosX(filteredAccel[0]*100000); //<>//
    plot.setPosY(filteredAccel[1]*100000);
    plot.setPosZ((filteredAccel[2])*100000);
    
    if (DEBUG) { //<>//
      print (" yaw " + degrees(yaw) + " pitch " + degrees(pitch) + " roll " + degrees(roll)); 
      //print("p " + degrees(pitch) + " y " + degrees(yaw) + " r " + degrees(roll));
      //print (" posX " + posX + " posY " + posY + " posZ " + posZ); 
      //print( " speedX " + oldSpeedX + " speedY " + oldSpeedY + " speedZ " + oldSpeedZ);
      //println( " gravity[0] " + gravity[0] + " gravity[1] " + gravity[1] + " gravity[2] " + gravity[2]); //<>//
     // println( " accelX     " + accelX     + " accelY     " + accelY     + " accelZ     " + accelZ);
     // println( " accelRX     " + accelXR     + " accelYR     " + accelYR     + " accelZR     " + accelZR);
      //print( " X " + gravity[0] + " Y " + gravity[1] + " z " + gravity[2]); //<>//
      println();                                                                          //<>//
    }                                                                                    
      //<>// //<>// //<>//                                                               
  } //<>//

  public float removeTempError(float temp) {
    if(abs(temp) > DATA_FILTER_MIN_EDGE && abs(temp) < DATA_FILTER_MAX_EDGE) {
      return temp;
    }
    else {
      return 0;
    }
  }

   public float getNewPos(float oldPos,float newSpeed) {
      return oldPos + newSpeed*deltaT;
   }
   
   public float getNewSpeed(float oldSpeed,float accel) {
      return oldSpeed + accel*deltaT;
   }


  float lowPassFilter(float data, float filterVal, float smoothedVal){
      return data * (1 - filterVal) + (smoothedVal  *  filterVal);
  }


// **************************************************
// ********** Set attribut for graphics *************
// **************************************************

    //public void setPosX(float p_posX) {
    //  viewObject3D.setPosX(p_posX);
    //  plot.setPosX(p_posX);
    //}
    
    // public void setPosY(float p_posY) {
    //  viewObject3D.setPosY(p_posY);
    //  plot.setPosY(p_posY);
    //}
    //public void setPosZ(float p_posZ) {
    //  viewObject3D.setPosZ(p_posZ);
    //  plot.setPosZ(p_posZ);
    //}
    public void setPitch(float p_Pitch) {
      
      viewObject3D.setPitch(p_Pitch);
      plot.setPitch(p_Pitch);
    }
    public void setRoll(float p_Roll) {
      viewObject3D.setRoll(p_Roll);
      plot.setRoll(p_Roll);
    }
    public void setYaw(float p_Yaw) {
      viewObject3D.setYaw(p_Yaw);
      plot.setYaw(p_Yaw);
    }

// ****************************************************
// ** Update object parameters with data from serial **
// ****************************************************
float getFloatFromSerial() {
  byte[] s = new byte[4];
  serialPort.readBytes(s);
  return ByteBuffer.wrap(s).order(ByteOrder.LITTLE_ENDIAN).getFloat();
}