#include <iostream>
#include <iomanip>
#include <bitset>
#include <string>
#define PI 3.14159265358979323846  /* pi */

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "std_srvs/Empty.h"

ros::Publisher pubIMU, pubMag, pubGPS, pubTemp, pubPres;


//Unused covariances initilized to zero's
boost::array<double, 9ul> linear_accel_covariance = { };
boost::array<double, 9ul> angular_vel_covariance = { };
boost::array<double, 9ul> orientation_covariance = { };
boost::array<double, 3ul> Antenna_A_offset = { };
boost::array<double, 3ul> baseline_position = { };
XmlRpc::XmlRpcValue rpc_temp;

// Custom user data to pass to packet callback function
struct UserData {
    int device_family;
};

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/util.h"
#include "vn/compositedata.h"
#include "vn/matrix.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Global Variables
int cont=0;
double flag1=0;
vec3d pos_o;

YawPitchRollMagneticAccelerationAndAngularRatesRegister reg;
QuaternionMagneticAccelerationAndAngularRatesRegister reg2;
vec3f reg3;
FilterMeasurementsVarianceParametersRegister reg4;
ImuMeasurementsRegister reg5;



// Method declarations for future use.
void asciiAsyncMessageReceived(void* userData, Packet& p, size_t index);
//void OtherSensors();

std::string frame_id;
// Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
bool tf_ned_to_enu;

// Basic loop so we can initilize our covariance parameters above
boost::array<double, 9ul> setCov(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    boost::array<double, 9ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 9; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}

// Basic loop so we can initilize our baseline position configuration above
boost::array<double, 3ul> setPos(XmlRpc::XmlRpcValue rpc){
    // Output covariance vector
    boost::array<double, 3ul> output = { 0.0 };

    // Convert the RPC message to array
    ROS_ASSERT(rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int i = 0; i < 3; i++){
        ROS_ASSERT(rpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        output[i] = (double)rpc[i];
    }
    return output;
}

int main(int argc, char *argv[]){

  // ROS node init
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
  pubMag = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
  pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
  pubTemp = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
  pubPres = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);

  // Serial Port Settings
  string SensorPort;
  int SensorBaudrate;
  int async_output_rate;

  // Sensor IMURATE (800Hz by default, used to configure device)
  int SensorImuRate;

  // Load all params
  // pn.param<type_of_data>(Param_name, Param_value, default_value)
  pn.param<std::string>("frame_id", frame_id, "vectornav");
  pn.param<bool>("tf_ned_to_enu", tf_ned_to_enu, false);
  pn.param<int>("async_output_rate", async_output_rate, 40);
  pn.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
  pn.param<int>("serial_baud", SensorBaudrate, 115200);
  pn.param<int>("fixed_imu_rate", SensorImuRate, 800);

  //Call to set covariances
  if(pn.getParam("linear_accel_covariance",rpc_temp)){
      linear_accel_covariance = setCov(rpc_temp);
  }
  if(pn.getParam("angular_vel_covariance",rpc_temp)){
      angular_vel_covariance = setCov(rpc_temp);
  }
  if(pn.getParam("orientation_covariance",rpc_temp)){
      orientation_covariance = setCov(rpc_temp);
  }

  //Call to set antenna A offset
  if(pn.getParam("Antenna_A_offset",rpc_temp)){
      Antenna_A_offset = setPos(rpc_temp);
  }
  //Call to set baseline position configuration
  if(pn.getParam("baseline_position",rpc_temp)){
      baseline_position = setPos(rpc_temp);
  }

  ROS_INFO("Connecting to: %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);
	// This example walks through using the VectorNav C++ Library to connect to
	// and interact with a VectorNav sensor.

/*
	// First determine which COM port your sensor is attached to and update the
	// constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate
	// constant below as well.
	// const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	// const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
	const uint32_t SensorBaudrate = 115200;
*/
	// Now let's create a VnSensor object and use it to connect to our sensor.
	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

  // Now we verify connection (Should be good if we made it this far)
  if(vs.verifySensorConnectivity()){
    ROS_INFO("Device connection established");
  }else{
    ROS_ERROR("No device communication");
  }

	// Let's query the sensor's model number.
	string mn = vs.readModelNumber();
  ROS_INFO("Model Number: %s\n", mn.c_str());

	// Let's do some simple reconfiguration of the sensor. As it comes from the
	// factory, the sensor outputs asynchronous data at 40 Hz.
  ROS_INFO("Current resgister configuration.....................................");
	vs.writeAsyncDataOutputFrequency(40); // see table 6.2.8
	uint32_t newHz = vs.readAsyncDataOutputFrequency();
  ROS_INFO("Async Frequency:\t%d Hz", newHz);

  //added by Harold
	ImuRateConfigurationRegister IMUR = vs.readImuRateConfiguration();
	IMUR.imuRate = 800;
	vs.writeImuRateConfiguration(IMUR);
	IMUR = vs.readImuRateConfiguration();
  ROS_INFO("IMU Frequency:\t\t%d Hz", IMUR.imuRate);

  ROS_INFO("...VPE Resgister....................................................");
	VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
	// Enable *********************************************************************
	vpeReg.enable = VPEENABLE_ENABLE;
	// Heading Mode ***************************************************************
  vpeReg.headingMode = HEADINGMODE_RELATIVE;
	// vpeReg.headingMode = HEADINGMODE_ABSOLUTE;
	// vpeReg.headingMode = HEADINGMODE_INDOOR;
	// Filtering Mode *************************************************************
	vpeReg.filteringMode = VPEMODE_MODE1;
	// Tuning Mode ****************************************************************
	// vpeReg.tuningMode = VPEMODE_OFF;
	vpeReg.tuningMode = VPEMODE_MODE1;
	vs.writeVpeBasicControl(vpeReg);
	vpeReg = vs.readVpeBasicControl();
  ROS_INFO("Enable:\t\t%d", vpeReg.enable);
  ROS_INFO("Heading Mode:\t%d", vpeReg.headingMode);
  ROS_INFO("Filtering Mode:\t%d", vpeReg.filteringMode);
  ROS_INFO("Tuning Mode:\t%d", vpeReg.tuningMode);
  ROS_INFO("...INS Resgister....................................................");
	InsBasicConfigurationRegisterVn300 InsReg = vs.readInsBasicConfigurationVn300();
	// Scenario *******************************************************************
	//InsReg.scenario = SCENARIO_AHRS;
	InsReg.scenario = SCENARIO_INSWITHPRESSURE;
	//InsReg.scenario = SCENARIO_INSWITHOUTPRESSURE;
	//InsReg.scenario = SCENARIO_GPSMOVINGBASELINEDYNAMIC;
	//InsReg.scenario = SCENARIO_GPSMOVINGBASELINESTATIC;
	// Ahrs Aiding ****************************************************************
	InsReg.ahrsAiding = 1;
	// Estimation Base line *******************************************************
	InsReg.estBaseline = 1;
	vs.writeInsBasicConfigurationVn300(InsReg); //added by Harold
	InsReg = vs.readInsBasicConfigurationVn300(); //added by Harold
  ROS_INFO("Scenario:\t%d", InsReg.scenario);
  ROS_INFO("AHRS Aiding:\t%d", InsReg.ahrsAiding);
  ROS_INFO("Base Line:\t%d", InsReg.estBaseline);

  ROS_INFO("...HIS Calibration..................................................");
	MagnetometerCalibrationControlRegister hsiReg = vs.readMagnetometerCalibrationControl();
	// HSI Mode *******************************************************************
	hsiReg.hsiMode = HSIMODE_RUN;
	//hsiReg.hsiMode = HSIMODE_OFF;
	// HSI Output *****************************************************************
	hsiReg.hsiOutput = HSIOUTPUT_USEONBOARD;
	//hsiReg.hsiOutput = HSIOUTPUT_NOONBOARD;
	vs.writeMagnetometerCalibrationControl(hsiReg);  //added by Harold
	hsiReg = vs.readMagnetometerCalibrationControl();//added by Harold
  ROS_INFO("Mode:\t%d", hsiReg.hsiMode);
  ROS_INFO("Output:\t%d\n", hsiReg.hsiOutput);

  ROS_INFO("BaseLine Configuration..............................................");
	/// BaseLine and Antenna A offset Configuration
	vs.writeGpsAntennaOffset({0,-0.18,0});
	vec3f Ant_offset = vs.readGpsAntennaOffset();
  ROS_INFO("Antena A offset:\t[%.2f, %.2f, %.2f]", Ant_offset[0],Ant_offset[1],Ant_offset[2]);
	GpsCompassBaselineRegister baseli_config = vs.readGpsCompassBaseline();
	baseli_config.position = {0, 0.35, 0};
  // Uncertainty calculation
  float max=0;
  for(int i = 0; i < 3; i++){
      if(baseli_config.position[i]>max){
        max = baseli_config.position[i];
      }
  }
  baseli_config.uncertainty = {max*0.025, max*0.025, max*0.025};
	// baseli_config.uncertainty = {0.00875, 0.00875, 0.00875 };
	vs.writeGpsCompassBaseline(baseli_config);
	baseli_config = vs.readGpsCompassBaseline();
  ROS_INFO("Position:\t\t[%.2f, %.2f, %.2f]", baseli_config.position[0], baseli_config.position[1], baseli_config.position[2]);
  ROS_INFO("Uncertainty:\t\t[%.4f, %.4f, %.4f]\n", baseli_config.uncertainty[0], baseli_config.uncertainty[1], baseli_config.uncertainty[2]);

	// LOOP

	while(1){
		if (!flag1){
			vs.writeAsyncDataOutputType(VNGPS);
		}
		else {
			// message type.
		  // vs.writeAsyncDataOutputType(VNYPR);
			// vs.writeAsyncDataOutputType(VNINE);
			vs.writeAsyncDataOutputType(VNINS);
		  // To try a different sensor output type, comment out the line for VNYPR and
		  // uncomment one of the GPS output types
		  //vs.writeAsyncDataOutputType(VNGPS);
		  //vs.writeAsyncDataOutputType(VNG2S);
		}
	  AsciiAsync asyncType = vs.readAsyncDataOutputType();
    ROS_INFO("ASCII Async Type: %d",asyncType);
		//cout << "ASCII Async Type: " << asyncType << endl;

		// VnSensor object.
		vs.registerAsyncPacketReceivedHandler(NULL, asciiAsyncMessageReceived);

		// cout << "Starting sleepubMagp..................................................................................................." << endl << endl;
		// Thread::sleepSec(5);
		if(flag1){
      ROS_INFO("Running.........................................................");
      while (ros::ok()){
        reg = vs.readYawPitchRollMagneticAccelerationAndAngularRates();
        reg2 = vs.readQuaternionMagneticAccelerationAndAngularRates();
        reg3 = vs.readVelocityCompensationMeasurement();
        reg4 = vs.readFilterMeasurementsVarianceParameters();
        reg5 = vs.readImuMeasurements();
        //reg6 = vs.readInsSolutionEcef();
        //cout << reg2.quat[0] <<" "<< reg2.quat[1] <<" "<< reg2.quat[2] <<" "<< reg2.quat[3] <<endl;
        //ros::spin(); // Need to make sure we disconnect properly. Check if all ok.
      }
		}else{
      ROS_INFO("Starting sleep..................................................");
			Thread::sleepSec(5);
		}
		// Unregister our callback method.
		vs.unregisterAsyncPacketReceivedHandler();
    ros::Duration(0.5).sleep();
	}
  // Node has been terminated
  vs.disconnect();
  ros::Duration(0.5).sleep();
	return 0;
}

void asciiAsyncMessageReceived(void* userData, Packet& p, size_t index){
  // vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  //UserData user_data = *static_cast<UserData*>(userData);

  // GPS
  sensor_msgs::NavSatFix msgGPS;
  msgGPS.header.stamp = ros::Time::now();
  msgGPS.header.frame_id = frame_id;
  // Magnetometer
  sensor_msgs::MagneticField msgMag;
  msgMag.header.stamp = msgGPS.header.stamp;
  msgMag.header.frame_id = msgGPS.header.frame_id;
  msgMag.magnetic_field.x = reg.mag[0];
  msgMag.magnetic_field.y = reg.mag[1];
  msgMag.magnetic_field.z = reg.mag[2];
  msgMag.magnetic_field_covariance[0] = reg4.magneticVariance[0];
  msgMag.magnetic_field_covariance[4] = reg4.magneticVariance[1];
  msgMag.magnetic_field_covariance[8] = reg4.magneticVariance[2];
  // IMU
  sensor_msgs::Imu msgIMU;
  msgIMU.header.stamp = msgGPS.header.stamp;
  msgIMU.header.frame_id = msgGPS.header.frame_id;
  msgIMU.orientation.x = reg2.quat[1];
  msgIMU.orientation.y = reg2.quat[0];
  msgIMU.orientation.z = -reg2.quat[2];
  msgIMU.orientation.w = reg2.quat[3];
  msgIMU.angular_velocity.x = reg2.gyro[0];
  msgIMU.angular_velocity.y = reg2.gyro[1];
  msgIMU.angular_velocity.z = -reg2.gyro[2];
  msgIMU.angular_velocity_covariance[0] = reg4.angularRateVariance[0];
  msgIMU.angular_velocity_covariance[4] = reg4.angularRateVariance[1];
  msgIMU.angular_velocity_covariance[8] = reg4.angularRateVariance[2];
  msgIMU.linear_acceleration.x = reg2.accel[0];
  msgIMU.linear_acceleration.y = reg2.accel[1];
  msgIMU.linear_acceleration.z = -reg2.accel[2];
  msgIMU.linear_acceleration_covariance[0] = reg4.accelerationVariance[0];
  msgIMU.linear_acceleration_covariance[4] = reg4.accelerationVariance[1];
  msgIMU.linear_acceleration_covariance[8] = reg4.accelerationVariance[2];
  // Temperature
  sensor_msgs::Temperature msgTemp;
  msgTemp.header.stamp = msgGPS.header.stamp;
  msgTemp.header.frame_id = msgGPS.header.frame_id;
  msgTemp.temperature = reg5.temp;
  // Pressure
  sensor_msgs::FluidPressure msgPres;
  msgPres.header.stamp = msgGPS.header.stamp;
  msgPres.header.frame_id = msgGPS.header.frame_id;
  msgPres.fluid_pressure = reg5.pressure;

  // Make sure we have an ASCII packet and not a binary packet.
  if(p.type() != Packet::TYPE_ASCII)
    return;

  // Make sure we have a VNYPR data packet.
  if(p.determineAsciiAsyncType() == VNYPR) {
    // We now need to parse out the yaw, pitch, roll data.
    vec3f ypr;
    p.parseVNYPR(&ypr);
  }

  // If the VNGPS output type was selected, print out that data
  else if(p.determineAsciiAsyncType() == VNGPS) {
		//std::cout << flag1 << '\n';
    double time;
    uint16_t week;
    uint8_t gpsFix;
    uint8_t numSats;
    vec3d lla;
    vec3f nedVel;
    vec3f nedAcc;
    float speedAcc;
    float timeAcc;
    p.parseVNGPS(&time, &week, &gpsFix, &numSats, &lla, &nedVel, &nedAcc, &speedAcc, &timeAcc);
		flag1 = lla[0];
  }

  // If the VNG2S output type was selected, print out that data
  else if(p.determineAsciiAsyncType() == VNG2S) {
    double time;
    uint16_t week;
    uint8_t gpsFix;
    uint8_t numSats;
    vec3d lla;
    vec3f nedVel;
    vec3f nedAcc;
    float speedAcc;
    float timeAcc;

    p.parseVNGPS(&time, &week, &gpsFix, &numSats, &lla, &nedVel, &nedAcc, &speedAcc, &timeAcc);
    //cout << "ASCII Async GPS2: " << lla << endl;
  }

  // If the VNINE output type was selected, print out that data
	else if(p.determineAsciiAsyncType() == VNINE) {
    double time;
    uint16_t week;
    uint16_t insStatus;
    vec3f ypr;
		vec3d pos;
		vec3f vel;
		float attun;
		float posun;
		float velun;

		p.parseVNINE(&time, &week, &insStatus, &ypr, &pos, &vel, &attun, &posun, &velun);
		// Status analysis
		/*cout << "Status:\t" << std::bitset<16>(insStatus) << endl;
		switch (insStatus&3) {
			case 0:
				cout << "Not tracking" << endl;
				break;
			case 1:
				cout << "Aligning." << endl;
				break;
			case 2:
				cout << "Tracking." << endl;
				break;
			case 4:
				cout << "Loss of GNSS" << endl;
				break;
		}
		if(insStatus&4){
			cout << "GNSS is fix" << endl;
		}else{
			cout << "GNSS is not fix" << endl;
		}
		if(!(insStatus&120)){
			cout << "Not Error" << endl;
		}else{
			if (insStatus&16) {
				cout << "IMU communication error is detected" << endl;
			}else if (insStatus&32){
				cout << "Magnetometer or Pressure sensor error is detected" << endl;
			}else if (insStatus&64){
				cout << "GNSS communication error is detected" << endl;
			}
		}
		if(insStatus&256){
			cout << "GNSS Compass solution is aiding the INS Filter heading" << endl;
		}else{
			cout << "GNSS Compass solution is not aiding the INS Filter heading" << endl;
		}
		if(insStatus&512){
			cout << "GNSS compass is operational and reporting a heading solution" << endl;
		}else{
			cout << "GNSS compass is not operational" << endl;
		}
*/
		// if(!cont)
		// 	pos_o = pos;
		// pos -= pos_o;
    cout << "ASCII Async ECEF_position:\t" << pos << ypr << endl << endl;
		// cout << "ASCII Async ECEF_status:\t" << std::bitset<16>(status) << endl;
     if(!cont)
		 	pos_o = pos;
    flag1 = pos[0];
		cont ++;
  }

	// If the VNINS output type was selected, print out that data
	else if(p.determineAsciiAsyncType() == VNINS) {
    double time;	// [seg]
    uint16_t week;
    uint16_t insStatus;
    vec3f ypr; 	// range: Yaw=+/- 180°; Pitch=+/- 90°; Roll=+/- 180°
		vec3d llh;	// Heigth_format: WGS84
		vec3f vel;	// [NED]
		float attun;
		float posun;
		float velun;

		p.parseVNINE(&time, &week, &insStatus, &ypr, &llh, &vel, &attun, &posun, &velun);

		// Status analysis
    /*
		cout << "Status:\t" << std::bitset<16>(insStatus) << endl;
		switch (insStatus&3) {
			case 0:
				cout << "Not tracking" << endl;
				break;
			case 1:
				cout << "Aligning." << endl;
				break;
			case 2:
				cout << "Tracking." << endl;
				break;
			case 4:
				cout << "Loss of GNSS" << endl;
				break;
		}
		if(insStatus&4){
			cout << "GNSS is fix" << endl;
		}else{
			cout << "GNSS is not fix" << endl;
		}
		if(!(insStatus&120)){
			cout << "Not Error" << endl;
		}else{
			if (insStatus&16) {
				cout << "IMU communication error is detected" << endl;
			}else if (insStatus&32){
				cout << "Magnetometer or Pressure sensor error is detected" << endl;
			}else if (insStatus&64){
				cout << "GNSS communication error is detected" << endl;
			}
		}
		if(insStatus&256){
			cout << "GNSS Compass solution is aiding the INS Filter heading" << endl;
		}else{
			cout << "GNSS Compass solution is not aiding the INS Filter heading" << endl;
		}
		if(insStatus&512){
			cout << "GNSS compass is operational and reporting a heading solution" << endl;
		}else{
			cout << "GNSS compass is not operational" << endl;
		}
*/
		// if(!cont)
		// 	pos_o = pos;
		// pos -= pos_o;
		// printf("LLH = [%.8f,%.8f,%.8f]\tYPR = [%.4f,%.4f,%.4f]\n",llh[0],llh[1],llh[2],ypr[0],ypr[1],ypr[2]);
		// file << std::setprecision(15) << llh[0] << "\t" << llh[1] << "\t" << llh[2] << "\t";
		// file << cont << "\t" << ypr[0] << "\t" << ypr[1] << "\t" << ypr[2] << "\n";
    msgGPS.status.status = insStatus;
    msgGPS.latitude = llh[0];
    msgGPS.longitude = llh[1];
    msgGPS.altitude = llh[2];
    // cout << ypr;
    //msgGPS.position_covariance = posun;

		cont ++;
  }
	// If the VNISE output type was selected, print out that data
	else if(p.determineAsciiAsyncType() == VNISE) {
    vec3f ypr;
		vec3d pos;
		vec3f vel;
		vec3f acc;
		vec3f w;

    p.parseVNISE(&ypr, &pos, &vel, &acc, &w);
		// if(!cont)
		// 	pos_o = pos;
		// pos -= pos_o;
		cont ++;
  }

		// If the VNGPE output type was selected, print out that data
		else if(p.determineAsciiAsyncType() == VNGPE) {
			double Tow;
			uint16_t week;
			uint8_t GNSSFix;
			uint8_t NumSats;
	    vec3d pos;
			vec3f vel;
			vec3f PAcc;
			float SAcc;
			float TimeAcc;

	    p.parseVNGPE(&Tow, &week, &GNSSFix, &NumSats, &pos, &vel, &PAcc, &SAcc, &TimeAcc);
			// if(!cont)
			// 	pos_o = pos;
			// pos -= pos_o;
	  }

  else {
    ROS_INFO("ASCII Async: Type(%d)......Not configurated",p.determineAsciiAsyncType());
    //cout << "ASCII Async: Type(" << p.determineAsciiAsyncType() << ")" << endl;
  }
  //OtherSensors();
  pubGPS.publish(msgGPS);
  pubMag.publish(msgMag);
  pubIMU.publish(msgIMU);
  pubTemp.publish(msgTemp);
  pubPres.publish(msgPres);
}
/*
void OtherSensors(){
  std::cout << "HOLA" << '\n';
}
*/
