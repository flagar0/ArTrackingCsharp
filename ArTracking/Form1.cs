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
                    #region Detect markers on last retrieved frame
                    VectorOfInt ids = new VectorOfInt(); // name/id of the detected markers
                    VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF(); // corners of the detected marker
                    VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF(); // rejected contours
                    ArucoInvoke.DetectMarkers(frame, ArucoDict, corners, ids, ArucoParameters, rejected);
                    #endregion

                    // If we detected at least one marker
                    if (ids.Size > 0)
                    {
                        
                        #region Draw detected markers
                        ArucoInvoke.DrawDetectedMarkers(frame, corners, ids, new MCvScalar(255, 0, 255));
                        #endregion


                        #region Estimate pose for each marker using camera calibration matrix and distortion coefficents
                        Mat rvecs = new Mat(); // rotation vector
                        Mat tvecs = new Mat(); // translation vector
                        ArucoInvoke.EstimatePoseSingleMarkers(corners, 80, cameraMatrix, distortionMatrix, rvecs, tvecs);
                        #endregion

                        #region Draw 3D orthogonal axis on markers using estimated pose
                        for (int i = 0; i < ids.Size; i++)
                        {
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
                                                     80 * 0.5f);
                                trans_x = tvecs.Data.ToString();
                            }
                        }
                        #endregion

                    }

                    #region Display current frame plus drawings
                    CvInvoke.Imshow("Image", frame);
                    CvInvoke.WaitKey(24);
                    CvInvoke.PutText(frame, ("translation_x: "+ trans_x), new Point(0, 60));
                    #endregion


                }

            }
        }



        private void button2_Click(object sender, EventArgs e)
        {
            recording = false;
            CvInvoke.DestroyAllWindows();
        }

        public void Calibracao()
        {
            #region Initialize Camera calibration matrix with distortion coefficients 
            // Calibration done with https://docs.opencv.org/3.4.3/d7/d21/tutorial_interactive_calibration.html
            String cameraConfigurationFile = "C:/Users/Interlab.INTERLAB-XPS/Documents/flavio/ARemC/ArTracking/ArTracking/cameraParameters.xml";
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


