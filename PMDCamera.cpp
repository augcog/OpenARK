#include "PMDCamera.h"

/***
Private constructor for the PMD depth sensor
***/
PMDCamera::PMDCamera(bool use_live_sensor)
{
	CONFIDENCE_THRESHHOLD = (60.0f / 255.0f*500.0f);
	INVALID_FLAG_VALUE = PMD_FLAG_INVALID;
	X_DIMENSION = 176;
	Y_DIMENSION = 120;
	//X_DIMENSION = 640;
	//Y_DIMENSION = 480;

	if (!use_live_sensor) {
		return;
	}

	int res;
	std::cout << "Trying to open pmd\n";
	res = pmdOpen(&hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM); //Open the PMD sensor
	if (res != PMD_OK)
	{
		pmdGetLastError(0, err, 128);
		fprintf(stderr, "Could not connect: %s\n", err);
		return;
	}
	printf("opened sensor\n");

	// Updating the sensor is necessary before any data can be retrieved
	res = pmdUpdate(hnd);
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Couldn't transfer data: %s\n", err);
		pmdClose(hnd);
		return;
	}
	printf("acquired image\n");

	// res: Structure which contains various meta-information about the data delivered by your Nano.
	// It is advisabe to always use the data delivered by this struct (for example the width and height of the imager
	// and the image format). Please refer to the PMSDSK documentation for more information	
	res = pmdGetSourceDataDescription(hnd, &dd);
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Couldn't get data description: %s\n", err);
		pmdClose(hnd);
		return;
	}

	printf("retrieved source data description\n");
	if (dd.subHeaderType != PMD_IMAGE_DATA)
	{
		fprintf(stderr, "Source data is not an image!\n");
		pmdClose(hnd);
		return;
	}

	numPixels = dd.img.numRows * dd.img.numColumns; // Number of pixels in camera
	dists = new float[3 * numPixels]; // Dists contains XYZ values. needs to be 3x the size of numPixels
	amps = new float[numPixels];
	frame = cvCreateImage(cvSize(dd.img.numColumns, dd.img.numRows), 8, 3); // Create the frame
	//mona frame.create(dd.img.numRows, dd.img.numColumns, CV_8UC3);
	KF.init(6, 3, 0);
	KF.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
													0, 1, 0, 0, 1, 0,
													0, 0, 1, 0, 0, 1,
													0, 0, 0, 1, 0, 0,
													0, 0, 0, 0, 1, 0,
													0, 0, 0, 0, 0, 1);
	measurement = cv::Mat_<float>::zeros(3, 1);

	//Initaite Kalman
	KF.statePre.setTo(0);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, cv::Scalar::all(.001)); // Adjust this for faster convergence - but higher noise
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
}

/***
Public deconstructor for he PMD depth sensor
***/
PMDCamera::~PMDCamera() {}

void PMDCamera::destroyInstance()
{
	printf("closing sensor\n");
	pmdClose(hnd);
	printf("sensor closed\n");
}

/***
Create xyzMap, zMap, ampMap, and flagMap from sensor input
***/
void PMDCamera::update()
{
	initilizeImages();
	
	fillInAmps();
	fillInZCoords();
	// Flags. Helps with denoising.
	
	unsigned *flags = new unsigned[ampMap.cols*ampMap.rows];
	int res = pmdGetFlags(hnd, flags, numPixels * sizeof(unsigned));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Couldn't get the flags: %s\n", err);
		pmdClose(hnd);
		return;
	}
	flagMap.data = (uchar *)flags;

    res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Couldn't update the PMD camera: %s\n", err);
		pmdClose(hnd);
		return;
	}
	delete(flags);
	
}


/***
Reads the depth data from the sensor and fills in the matrix
***/
void PMDCamera::fillInZCoords()
{
	int res = pmdGet3DCoordinates(hnd, dists, 3 * numPixels * sizeof(float)); //store x,y,z coordinates dists (type: float*)
	//float * zCoords = new float[1]; //store z-Coordinates of dists in zCoords
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Couldn't get 3D coordinates: %s\n", err);
		pmdClose(hnd);
		return;
	}
	xyzMap = cv::Mat(xyzMap.size(), xyzMap.type(), dists);
	//std::cout << "nrows: " << xyzMap.rows << std::endl;
	//std::cout << "ncols: " << xyzMap.cols << std::endl;
}

/***
Reads the amplitude data from the sensor and fills in the matrix
***/
void PMDCamera::fillInAmps()
{
	int res = pmdGetAmplitudes(hnd, amps, numPixels * sizeof(float));
	//float * dataPtr = amps;
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Couldn't get amplitudes: %s\n", err);
		pmdClose(hnd);
		return;
	}
	ampMap.data = (uchar *)amps;
}

/***
Returns the X value at (i, j)
***/
float PMDCamera::getX(int i, int j) const
{
	int flat = j * dd.img.numColumns * 3 + i * 3;
	return dists[flat];
}

/***
Returns the Y value at (i, j)
***/
float PMDCamera::getY(int i, int j) const
{
	int flat = j * dd.img.numColumns * 3 + i * 3;
	return dists[flat + 1];
}

/***
Returns the Z value at (i, j)
***/
float PMDCamera::getZ(int i, int j) const
{
	int flat = j * dd.img.numColumns * 3 + i * 3;
	return dists[flat + 2];
}