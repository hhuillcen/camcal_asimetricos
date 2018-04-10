#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;


Mat src, src_gray;
int thresh = 140;
int max_thresh = 255;
int frames_correctos=0;


Mat temp = Mat::zeros( 480, 640, CV_8UC3 );
Mat temp1 = Mat::zeros( 480, 640, CV_8UC3 );
vector<vector<Point2f>> coord2D;
float dist_x = 35;
float dist_y = 35;
vector<Point3f> puntos3D;
vector<vector<Point3f>> coord3D;
int capturas=0;

Mat intrinsic = Mat(3, 3, CV_32FC1);
Mat distCoeffs;
vector<Mat> rvecs;
vector<Mat> tvecs;

Size patternsize(4,11);

int total_puntos_computados=0;
double distancia=0;
double total_distancias=0;

int nro_imagenes=25;
int cont_imag=0;
int puntos_patron=44;

double distancia_punto_recta(Point2f p1,Point2f p2, Point2f clave)
{
		double m=(p2.y-p1.y)/(p2.x-p1.x);
		double b=p1.y-(m*p1.x);
		double dist=(abs(m*clave.x-clave.y+b)) / (sqrt((m*m)+1));
		return dist;
}


static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs
         )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    //perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
		//cout<<" \n objectpoints "<<objectPoints[i]<<"\n"<<endl;
		//cout<<" \n imagepoints2 "<<imagePoints2[i]<<"\n"<<endl;
		//
				for (int x=0;x<=imagePoints2.size();x++)   // muestra los puntos detectados en cada frame
				{
				ostringstream convert; 
				convert<<x;
				circle(temp1, imagePoints2[x],1, Scalar(0,0,255),CV_FILLED, 8,0);
				putText(temp1, convert.str(), imagePoints2[x], FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
				}
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

		//cout<<" \n imagepoint de : "<<i<<"\n"<<imagePoints2[i]<<"\n"<<endl;
		namedWindow( "Temporal1", CV_WINDOW_AUTOSIZE );
		imshow( "Temporal1", temp1 );
		//waitKey();
        int n = (int)objectPoints[i].size();
        totalErr += err*err;
        totalPoints += n;
    }

    return sqrt(totalErr/totalPoints);
}


static double computeColiErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs
         )
{
    vector<Point2f> imagePoints2;
	int i;
    

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		
		//colinearidad entre 0 y 3
		for (int x=1; x<=2;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[0],imagePoints2[3],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 4 y 7
		for (int x=5; x<=6;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[4],imagePoints2[7],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 8 y 11
		for (int x=9; x<=10;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[8],imagePoints2[11],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 12 y 15
		for (int x=13; x<=14;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[12],imagePoints2[15],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 16 y 19
		for (int x=17; x<=18;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[17],imagePoints2[19],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 20 y 23
		for (int x=21; x<=22;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[20],imagePoints2[23],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 24 y 27
		for (int x=25; x<=26;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[24],imagePoints2[27],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 28 y 31
		for (int x=29; x<=30;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[28],imagePoints2[31],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 32 y 35
		for (int x=33; x<=34;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[32],imagePoints2[35],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

		//colinearidad entre 36 y 39
		for (int x=37; x<=38;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[37],imagePoints2[39],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}
		//colinearidad entre 40 y 43
		for (int x=41; x<=42;x++)
		{
			distancia= distancia_punto_recta(imagePoints2[40],imagePoints2[43],imagePoints2[x]);
			total_distancias=total_distancias+distancia;
			total_puntos_computados=total_puntos_computados+1;
		}

    }

    return total_distancias/total_puntos_computados;
}






/** @function reconocer_elipses */
void reconocer_elipses(int, void*) 
{
 // Mat threshold_output;
//  vector<vector<Point> > contours;
//  vector<Vec4i> hierarchy;
  vector<Point2f> puntos;

   


	//parámetros para findCirclesGrid

	SimpleBlobDetector::Params params;
	params.thresholdStep = 10;
	params.minThreshold = 100;
	params.maxThreshold = 255;
	params.minRepeatability = 1;
	params.maxArea = 10000; // 100 * 100
	params.minArea = 50; // 10 * 10
	Ptr<FeatureDetector> blobDetector = new SimpleBlobDetector(params);


	bool deteccion;
			

	//EN CASO DE USAR EL PATRON DE CIRCULOS
	deteccion = findCirclesGrid(src_gray, patternsize, puntos, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector);

	//EN CASO DE USAR EL PATRON DE TABLERO DE AJEDREZ
	//deteccion = findChessboardCorners(src_gray,patternsize,puntos);


	if (deteccion == false )
	{
		cout << "la deteccion fallo!" << endl;
		//return 0;
	}
	else
	{


		if (puntos.size()==puntos_patron)
		//if(coli<=0.2)// && sep1>3 && sep2>3 && sep3>3 && sep4>3 && sep5>3 && sep6>3 && sep7>3 && sep8>3 && sep9>3)
		{
			cout << "Exitoso!" << endl;
			drawChessboardCorners(src,patternsize,puntos,true);   //dibuja los centros y las lineas de colores

			for (int x=0;x<=puntos.size();x++)   // muestra los puntos detectados en cada frame
			{
				//ostringstream convert; 
				//convert<<x;
				circle(temp, puntos[x],1, Scalar(0,0,255),CV_FILLED, 8,0);
				//putText(temp, convert.str(), puntos[x], FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
			}
			namedWindow( "Temporal", CV_WINDOW_AUTOSIZE );
			imshow( "Temporal", temp );
			//waitKey();
			//grabar las coordenadas 2D 

			coord2D.push_back(puntos);
			capturas=capturas+1;
		  imshow( "Original", src );
		  cont_imag=cont_imag+1;
		  cout<<"imagen valida nro: "<<cont_imag<<endl;
		  waitKey(30);


		  }
	}
}








/** @function main */
int main( int argc, char** argv )
{


	//for(int i=0; i<patternsize.height;i++){
	//	for(int j=0; j<patternsize.width;j++) {
	//		puntos3D.push_back(Point3f(j*dist_x, i*dist_y, 0)); //adiciona un elemento al final
	//	}
	//}

	 for( int i = 0; i < patternsize.height; i++ )
		{
		for( int j = 0; j < patternsize.width; j++ )
			{
			puntos3D.push_back(Point3f(float((2*j + i % 2)*dist_x), float(i*dist_y), 0));
			}
		}

	//intrinsic.ptr<float>(0)[0] = 1;
 //   intrinsic.ptr<float>(1)[1] = 1;

	namedWindow( "OpenCV Video", CV_WINDOW_AUTOSIZE);

	// cargar el archivo de video especificado
	VideoCapture vc("d://casa//Asymmetric1.mp4");


	// verificar si se ha podio cargar el video
	if(!vc.isOpened()) return -1;
	// obtener los cuadros por segundo
	//double fps = vc.get(CV_CAP_PROP_FPS);

	//cout<<"velocidad es: "<<fps<<endl;

	//int c=0;g
	//int a=1;
	//int pivote=147;

	int frnb ( vc.get ( CV_CAP_PROP_FRAME_COUNT ) );
    std::cout << "Numero de Frames = " << frnb << endl;

	//waitKey();

	int leer_frame;

	while (true)
	{

		//Mat frame;
		//vc >> frame;
		//c=c+1;
		//srand(time(NULL));

		leer_frame=1+rand()%(frnb-1);
		//waitKey();
		Mat frame;
		vc.set ( CV_CAP_PROP_POS_FRAMES , leer_frame );
		vc.read(frame);

		imshow( "Original", frame);

		//if ((c==1)||(c==50)||(c==100)||(c==150)||(c==155)||(c==220)||(c==274)||(c==280)
		//	||(c==300)||(c==320)||(c==380)||(c==390)||(c==440)||(c==470)||(c==480)||(c==500)
		//	||(c==545)||(c==560)||(c==590)||(c==610)||(c==620)||(c==660)||(c==680)||(c==700)||(c==720))
		//if (c==pivote*a)

		if(cont_imag<nro_imagenes)
		{
			cout<<endl<<"PROCESAMIENTO DE FRAME NRO :  "<< leer_frame <<endl<<endl;

			src = frame.clone();

			/// Convertir imagen a escala de grises y luego suavizar
			cvtColor( src, src_gray, CV_BGR2GRAY );
			//blur( src_gray, src_gray, Size(3,3) );


			//imshow( "source", src_gray );

			//crear la barra de segmentacion
			//createTrackbar( " Umbral de segmemtación:", "source", &thresh, max_thresh, reconocer_elipses );
			reconocer_elipses(0,0);
			//a=a+1;
			
			//waitKey();
		}

		if(cont_imag==nro_imagenes)
		{
			//calibrar la camara
			cout<<endl<<"EMPEZANDO CALIBRACION CON  :  "<< cont_imag <<"  imagenes correctamente detectadas"<<endl;
			  //destroyWindow("Reconocimiento");
			  //destroyWindow("Temporal");
			  coord3D.resize(cont_imag,puntos3D);
			  //Mat distCoeffs;
			  //vector<Mat> rvecs, tvecs;
				  //Intrínsecos:
				//Mat cameraMatrix;//(3,3,CV_64FC1);
				//Mat distCoeffs;//(8,1,CV_64FC1);
				//Extrínsecos:
				
			 
				//cameraMatrix = initCameraMatrix2D(coord3D, coord2D, Size(5,4));
				//double rms = cv::calibrateCamera(coord3D, coord2D, Size(5,4), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_RATIONAL_MODEL);
				intrinsic = initCameraMatrix2D(coord3D, coord2D,src.size());	
				calibrateCamera(coord3D, coord2D,src.size(), intrinsic, distCoeffs, rvecs, tvecs);
				
				cout<<"Coeficientes de Distorsión \n"<<distCoeffs<<endl;

				cout<<"Matriz de la camara \n"<<intrinsic<<endl;
				waitKey();

				//Calcular el error
				//vector<float> reprojErrs;

				//
				double totalAvgErr = computeReprojectionErrors(coord3D, coord2D, rvecs, tvecs, intrinsic, distCoeffs);

				cout << "\n Error standart de prediccion promedio: " << totalAvgErr<< endl;
				waitKey();

				double totalColiErr = computeColiErrors(coord3D, coord2D, rvecs, tvecs, intrinsic, distCoeffs);

				cout << "\n Promedio de colinearidad de los puntos " << totalColiErr<<endl;
				waitKey();

				cout << "\n Pulse una tecla para empezar con los resultados.." << endl;
				waitKey();

				cont_imag=cont_imag+1;
				//vector<Point2f> projectedPoints;
				//projectPoints(coord3D[2], rvecs[2], tvecs[2], intrinsic, distCoeffs, projectedPoints);
				//for (int i = 0; i < projectedPoints.size(); ++i)
				//{
				//	cout << "Image point: " << coord2D[i] << " Projected to " << projectedPoints[i] << endl;
				//}			
				//waitKey();
				
				//VideoCapture cap(0);  //apertura la webcam
				//if(!cap.isOpened())
				//	return -1;


				//for(;;)
				//{
				//	Mat frame1;
				//	cap >> frame1;
				//	Mat imageUndistorted;
				//	undistort(frame1, imageUndistorted, intrinsic, distCoeffs);
				//	imshow( "Original", frame1);
				//	imshow("Calibrado", imageUndistorted);
				//	waitKey(30);

				//}
		}

		if (cont_imag>nro_imagenes)
		{
				VideoCapture vc1("d://casa//Asymmetric1.mp4");
				if(!vc1.isOpened()) return -1;
				for(;;)
				{
					Mat frame1;
					vc1>> frame1;
					Mat imageUndistorted;
					undistort(frame1, imageUndistorted, intrinsic, distCoeffs);
					imshow( "Original", frame1);
					imshow("Calibrado", imageUndistorted);
					waitKey(30);

				}

		}


	  
	}
	
}






