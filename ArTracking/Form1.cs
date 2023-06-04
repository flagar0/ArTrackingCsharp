using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Emgu.CV; 
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Numpy;

namespace ArTracking
{
    public partial class Form1 : Form
    {
        VideoCapture capture;
        Dictionary ArucoDict;
        DetectorParameters ArucoParameters;
        public bool recording = false;
        Mat cameraMatrix, distortionMatrix;
        public string trans_x;

        public Form1()
        {
            InitializeComponent();
        }

        public void Form1_Load(object sender, EventArgs e)
        {
            #region Initialize video capture object on default webcam (0)
            // Instantiate a webcam abstraction

            capture = new VideoCapture(0);

            ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict6X6_250);

            ArucoParameters = new DetectorParameters();
            ArucoParameters = DetectorParameters.GetDefault();
            ArucoParameters.CornerRefinementMethod = DetectorParameters.RefinementMethod.Contour;


            Calibracao();
            #endregion
        }


        private void button1_Click(object sender, EventArgs e)
        {
            recording = true;
            while (recording)
            {
                
                #region Capture a frame with webcam
                Mat frame = new Mat();
                frame = capture.QueryFrame();
                #endregion

                if (!frame.IsEmpty)
                {
                    #region Detect markers 
                    VectorOfInt ids = new VectorOfInt(); 
                    VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); 
                    VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF(); 
                    ArucoInvoke.DetectMarkers(frame, ArucoDict, corners, ids, ArucoParameters, rejected); ;
                    #endregion

                    
                    if (ids.Size > 0)
                    {
                        
                        #region Draw detected markers
                        ArucoInvoke.DrawDetectedMarkers(frame, corners,ids, new MCvScalar(255, 0, 255));
                        #endregion


                        #region Estimate pose of marker
                        Mat rvecs = new Mat(); // rotation vector
                        Mat tvecs = new Mat(); // translation vector
                        ArucoInvoke.EstimatePoseSingleMarkers(corners, (float)13.2, cameraMatrix, distortionMatrix, rvecs, tvecs);
                        #endregion

                        #region Draw 3D orthogonal axis on markers using estimated pose
                        for (int i = 0; i < ids.Size; i++)
                        {
                            //Mat choosen_marker_position = getPositionMatrix(rvecs.Row(i), tvecs.Row(i));
                            using (Mat rvecMat = rvecs.Row(i))
                            using (Mat tvecMat = tvecs.Row(i))
                            using (VectorOfDouble rvec = new VectorOfDouble())
                            using (VectorOfDouble tvec = new VectorOfDouble())
                            {
                                double[] values = new double[3];
                                rvecMat.CopyTo(values);
                                rvec.Push(values);
                                tvecMat.CopyTo(values);
                                tvec.Push(values);
                                ArucoInvoke.DrawAxis(frame,
                                                     cameraMatrix,
                                                     distortionMatrix,
                                                     rvec,
                                                     tvec,
                                                     5);
                                trans_x = tvec[0].ToString();
                            }
                        }
                        #endregion

                    }

                    #region Display current frame plus drawings
                    CvInvoke.PutText(frame, ("translation_x: " + trans_x), new Point(0, 60), Emgu.CV.CvEnum.FontFace.HersheySimplex, 0.6, new MCvScalar(0, 255, 0), 2, LineType.Filled);
                    CvInvoke.Imshow("Image", frame);
                    CvInvoke.WaitKey(24);
                   
                    #endregion


                }

            }
        }



        private void button2_Click(object sender, EventArgs e)
        {
            recording = false;
            CvInvoke.DestroyAllWindows();
        }

        public Mat getPositionMatrix(Mat rvec, Mat tvec)
        {
            #region Calculo para distancia do cubo

            return null;
            #endregion
        }

        public void Calibracao()
        {
            #region Initialize Camera calibration matrix with distortion coefficients 
            // Calibration done with https://docs.opencv.org/3.4.3/d7/d21/tutorial_interactive_calibration.html
            String cameraConfigurationFile = "C:/Users/Flavio Midea/Downloads/InterLab/ArCsharp/ArTrackingCsharp/ArTracking/cameraParameters.xml";
            FileStorage fs = new FileStorage(cameraConfigurationFile, FileStorage.Mode.Read);
            if (!fs.IsOpened)
            {
                Console.WriteLine("Could not open configuration file " + cameraConfigurationFile);
                return;
            }   
             cameraMatrix = new Mat(new Size(3, 3), DepthType.Cv32F, 1);
             distortionMatrix = new Mat(1, 8, DepthType.Cv32F, 1);
            fs["cameraMatrix"].ReadMat(cameraMatrix);
            fs["dist_coeffs"].ReadMat(distortionMatrix);
            #endregion
        }


    }
    } 


