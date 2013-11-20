/*
 #include <inttypes.h>
 #include <termios.h>
 #include <stdlib.h>
 #include <stdio.h>
 #include <unistd.h>
 #include <fcntl.h>
 #include <sys/types.h>
 #include <sys/stat.h>

 #include <sys/time.h>
 #include <time.h>
 #include <unistd.h>
 #include <string.h>
 */

#define FT double

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>

#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

#include <openAHRS/util/util.h>
#include <openAHRS/util/net.h>

#include <openAHRS/kalman/UKFst7.h>

#include <openAHRS/util/octave.h>
#include <openAHRS/util/net.h>

#ifndef B921600
#define	B921600	0000026
#endif /* B921600 */

#define PORT1 "/dev/ttyUSB0"

#define BAUD  B500000 //B921600
struct IMUData
{
	double sup;

	double gyroX;
	double gyroY;
	double gyroZ;

	double acclX;
	double acclY;
	double acclZ;

	double tempX;
	double tempY;
	double tempZ;
};

static const FT meas_variance = 0.01;

char data[50];
struct IMUData imu;

openAHRS::UKFst7 K7;

int zxc = 0;

struct timespec ts1, ts2;

static openAHRS::util::UDPConnection udp( "127.0.0.1", 4444 );

int initPort(char* dev)
{
	int fd;
	struct termios oldtio;
	/// Initialize serial port ///

	fd = open(dev, O_RDWR | O_NOCTTY /*| O_NDELAY*/);

	tcgetattr(fd, &oldtio);

	// set up new settings
	struct termios newtio;
	memset(&newtio, 0, sizeof(newtio));
	newtio.c_cflag = CS8 | CLOCAL | CREAD;
	newtio.c_iflag = INPCK; //IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0) {
		fprintf(stderr, "Failed to set serial baud rate: %d\n", BAUD);
		tcsetattr(fd, TCSANOW, &oldtio);
		close(fd);
		fd = -1;
		return fd;
	}

	// activate new settings
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	return fd;
}

int16_t getShort14(char * dat)
{
	int16_t val = 0;

	((char *) &val)[0] = dat[1];
	((char *) &val)[1] = dat[0];

	val = val & 0x3FFF;

	if (val > (16383 / 2 - 1))
		val = val - 16383;
	return val;
}

int16_t getShort12(char * dat)
{
	int16_t val = 0;

	((char *) &val)[0] = dat[1];
	((char *) &val)[1] = dat[0];

	val = val & 0x0FFF;

	if (val > (4095 / 2 - 1))
		val = val - 4095;
	return val;
}

void interpret(char * dat)
{
	int16_t sup, gyroX, gyroY, gyroZ, acclX, acclY, acclZ, tempX, tempY, tempZ;

	Matrix<FT,3,1> accels;
	Matrix<FT,3,1> angles;
	Matrix<FT,3,1> gyros;
	Matrix<FT,7,1> st;

	sup = getShort12(&dat[0]);

	gyroX = getShort14(&dat[2]);
	gyroY = getShort14(&dat[4]);
	gyroZ = getShort14(&dat[6]);

	acclX = getShort14(&dat[8]);
	acclY = getShort14(&dat[10]);
	acclZ = getShort14(&dat[12]);

	tempX = getShort12(&dat[14]);
	tempY = getShort12(&dat[16]);
	tempZ = getShort12(&dat[18]);

	imu.gyroX = gyroX * 0.05 * C_PI / 180.0;
	imu.gyroY = gyroY * 0.05 * C_PI / 180.0;
	imu.gyroZ = gyroZ * 0.05 * C_PI / 180.0;

	imu.acclX = acclX * 0.000333;
	imu.acclY = acclY * 0.000333;
	imu.acclZ = acclZ * 0.000333;

	imu.tempX = 25.0 + tempX * 0.136;
	imu.tempY = 25.0 + tempY * 0.136;
	imu.tempZ = 25.0 + tempZ * 0.136;

	// if(acclZ == 0)
	//   printf("gyro [deg/s] %7.2f %7.2f %7.2f  accl [g] %7.4f %7.4f %7.4f temp [degC] %7.2f %7.2f %7.2f \n", imu.gyroX, imu.gyroY, imu.gyroZ, imu.acclX, imu.acclY, imu.acclZ, imu.tempX, imu.tempX, imu.tempX);

	accels(0) = imu.acclX;
	accels(1) = imu.acclY;
	accels(2) = imu.acclZ;

	gyros(0) = imu.gyroX;
	gyros(1) = imu.gyroY;
	gyros(2) = imu.gyroZ;

	openAHRS
	::util::accelToPR( accels, angles);

	clock_gettime(CLOCK_REALTIME, &ts2);
	double dt = (float) (1.0 * (1.0 * ts2.tv_nsec - ts1.tv_nsec * 1.0) * 1e-9 + 1.0 * ts2.tv_sec - 1.0 * ts1.tv_sec );

	ts1 = ts2;

	K7.KalmanUpdate(zxc, angles, dt);
	K7.getStateVector(st);
	K7.KalmanPredict(zxc++, gyros, dt);

	Matrix<FT,3,1> angles2 = openAHRS::util::quatToEulerNorm( st.start<4>() );
	printf("RPY %6.3f %6.3f %6.3f dt %6.3f \n", angles2(0), angles2(1), angles2(2), dt);

	static char netStr[1024];
	sprintf(netStr, "Roll: %lf Pitch: %lf Yaw: %lf Bias1: %lf Bias2: %lf Bias3: %lf Ax: %lf Ay: %lf Az: %lf RH: %lf", angles2(0), angles2(1), angles2(2), st(4), st(5), st(6), accels(0), accels(1), accels(2), 0.0);

	udp.Send(netStr, strlen(netStr));
}

int main()
{
	int fd1;

	Matrix<FT,3,1> angle;
	Matrix<FT,3,1> startBias;

	fd1 = initPort(PORT1);

	angle(0, 0) = 0.0;
	angle(1, 0) = 0.0;
	angle(2, 0) = 0.0;

	startBias(0, 0) = 0.0;
	startBias(1, 0) = 0.0;
	startBias(2, 0) = 0.0;

	printf("Locale is: %s\n", setlocale(LC_ALL, ""));

	K7.KalmanInit(angle, startBias, meas_variance, 1e-2, 1e-5);
	clock_gettime(CLOCK_REALTIME, &ts1);
	int xxx = 0;
	// do magic
	for (;;) {
		int dlen = 0;

		fd_set rfds;

		FD_ZERO(&rfds);
		FD_SET(fd1, &rfds);

		// timeout
		struct timeval timeout;
		timeout.tv_sec = (time_t) 0;
		timeout.tv_usec = 50000;

		int select_retval = select(fd1 + 1, &rfds, NULL, NULL, &timeout);

		if (select_retval == 0) {
			printf("timeout !!! \n");
		} else {
			int ret = read(fd1, data + dlen, 50 - dlen);
			if (ret <= 0)
				continue;
			dlen += ret;
			for (unsigned int i = 0; i < dlen; i++) {
				if (data[i] == '\n') {
					if (i > 21)
						interpret(data + i - 22);
					unsigned int k = 0;

					for (unsigned int j = i + 1; j < dlen; j++) {
						data[k++] = data[j];
					}

					dlen = k;

					i = 0;
				}
			}
		}
	}

	/// restore serial settings ///
	//if (fd1 > 0)
	// tcsetattr(fd1, TCSANOW, &oldtio);
	close(fd1);

	return 0;
}
