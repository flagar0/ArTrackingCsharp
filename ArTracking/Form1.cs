using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Emgu.CV; 
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using NumSharp;

namespace ArTracking
{
    public partial class Form1 : Form
    {
        VideoCapture capture;
        Dictionary ArucoDict;
        DetectorParameters ArucoParameters;
        public bool recording = false;
        Mat cameraMatrix, distortionMatrix;
        Dictionary<string,string> Dados = new Dictionary<string, string>() {
            {"translation_x", ""},
            {"translation_y", ""},
            {"translation_z", ""}


        };

        public Form1()
        {
            InitializeComponent();
        }




        public void Form1_Load(object sender, EventArgs e)
        {
            #region Initialize video capture object on default webcam (0)
            // Instantiate a webcam abstraction

            capture = new VideoCapture();

            ArucoDict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict6X6_250);

            ArucoParameters = new DetectorParameters();
            ArucoParameters = DetectorParameters.GetDefault();


            Calibracao();
            #endregion
        }


        private void button1_Click(object sender, EventArgs e)
        {
            recording = true;
            Tracking();
        }



        private void button2_Click(object sender, EventArgs e)
        {
            recording = false;
            CvInvoke.DestroyAllWindows();
        }

        public void Tracking()
        {
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
                                var rvecND = new NDArray(values);

                                tvecMat.CopyTo(values);
                                tvec.Push(values);
                                var tvecND = new NDArray(values);


                                ArucoInvoke.DrawAxis(frame,
                                                     cameraMatrix,
                                                     distortionMatrix,
                                                     rvec,
                                                     tvec,
                                                     80 * 0.5f);

                                #region Guarda os valores de translacao
                                NDArray matrixPosition = GetPositionMatrix(rvecND,tvecND);
                                Dados["translation_x"] = rvecND[0].ToString();
                                Dados["translation_y"] = rvecND[1].ToString();
                                Dados["translation_z"] = rvecND[2].ToString();
                                #endregion
                            }
                        }
                        #endregion
                   }


                    MostraVideos(frame);

                }

            }
        }



        public void MostraVideos(Mat frame)
        {
            #region Mostra da tela de tracking
            CvInvoke.NamedWindow("Tracking", WindowFlags.Normal);
            CvInvoke.PutText(frame, ("translation_x: " + Dados["translation_x"]), new Point(0, 60), Emgu.CV.CvEnum.FontFace.HersheySimplex, 0.5, new MCvScalar(0, 255, 0), 2, LineType.Filled);
            CvInvoke.PutText(frame, ("translation_y: " + Dados["translation_y"]), new Point(0, 80), Emgu.CV.CvEnum.FontFace.HersheySimplex, 0.5, new MCvScalar(0, 255, 0), 2, LineType.Filled);
            CvInvoke.PutText(frame, ("translation_z: " + Dados["translation_z"]), new Point(0, 100), Emgu.CV.CvEnum.FontFace.HersheySimplex, 0.5, new MCvScalar(0, 255, 0), 2, LineType.Filled);
            CvInvoke.Imshow("Tracking", frame);
            CvInvoke.WaitKey(24);
    

            #endregion
        }


        public NDArray GetPositionMatrix(NDArray rvec, NDArray tvec)
        {
            var rotMtx = new Mat(3, 3, DepthType.Cv64F, 1);
            double[] values = new double[3];
            double[] values2 = new double[3];
            rvec.CopyTo(values);
            VectorOfDouble rvecVec = new VectorOfDouble();
            
            rvecVec.Push(values);
            CvInvoke.Rodrigues(rvecVec, rotMtx);

            rotMtx.CopyTo(values2);
            
            NDArray rotMtxND = new NDArray(values2);
            var position = np.concatenate(new NDArray[] { rotMtxND, np.transpose(tvec)});

            var zeroRow = new NDArray(new double[,] { { 0, 0, 0, 1 } });
            position = np.concatenate(
                new NDArray[] { position, zeroRow });


            return position;

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


