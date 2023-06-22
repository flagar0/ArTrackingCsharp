using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using NumSharp;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using OpenCvSharp.Internal.Vectors;
//using NumsharpOpencvSharpConvertor;

namespace ArTracking
{
    public partial class Form1 : Form
    {
        VideoCapture capture;
        Dictionary ArucoDict;
        DetectorParameters ArucoParameters;
        public bool recording = false;
        Mat cameraMatrix, distortionMatrix;

        Dictionary<string, string> Dados = new Dictionary<string, string>() {
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

            capture = new VideoCapture(0);

            ArucoDict = CvAruco.GetPredefinedDictionary(PredefinedDictionaryName.Dict6X6_250);

            ArucoParameters = new DetectorParameters();
            //ArucoParameters = DetectorParameters.
            Calibracao();
            #endregion
        }


        private void button1_Click(object sender, EventArgs e)
        {
            recording = true;
            Tracking();
        }

        /// <summary>
        /// Convert a rotation vector in a rotation matrix using Rodrigues algorithm.
        /// </summary>
        /// <param name="rvec">The rotation vector to convert.</param>
        /// <returns></returns>
        Mat GetRotationMatrixFromRotationVector(Mat rvec)
        {
            Mat rmat = new Mat();
            Cv2.Rodrigues(rvec, rmat);
            return rmat;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            recording = false;
            Cv2.DestroyAllWindows();
        }

        public void Tracking()
        {
            while (recording)
            {

                #region Capture a frame with webcam
                Mat frame = new Mat();
                capture.Read(frame);
                #endregion

                if (!frame.IsDisposed)
                {
                    #region Detect markers on last retrieved frame
                    int[] ids = new int[0]; ; // name/id of the detected markers
                    Point2f[][] corners; // corners of the detected marker
                    Point2f[][] rejected; // rejected contours
                    CvAruco.DetectMarkers(frame, ArucoDict, out corners, out ids, ArucoParameters, out rejected);
                    #endregion

                    // If we detected at least one marker
                    if (ids.Length > 0)
                    {

                        #region Draw detected markers
                        CvAruco.DrawDetectedMarkers(frame, corners, ids, new Scalar(255, 0, 255));
                        #endregion


                        #region Estimate pose for each marker using camera calibration matrix and distortion coefficents
                        Mat rvecs = new Mat(); // rotation vector
                        Mat tvecs = new Mat(); // translation vector
                        CvAruco.EstimatePoseSingleMarkers(corners, 80, cameraMatrix, distortionMatrix, rvecs, tvecs);
                        #endregion

                        #region Draw 3D orthogonal axis on markers using estimated pose


                        for (int i = 0; i < ids.Length; i++)
                        {
                            using (Mat rvecmat = rvecs.Row(i))
                            using (Mat tvecmat = tvecs.Row(i))
                            {
                                var positionMatrix = getPositionMatrix(rvecmat, tvecmat);



                                Cv2.DrawFrameAxes(frame,
                                                     cameraMatrix,
                                                     distortionMatrix,
                                                     rvecmat,
                                                     tvecmat,
                                                     80 * 0.5f);

                                #region guarda os valores de translacao
                                // Dados["translation_x"] = tvecmat.;
                                // Dados["translation_y"] = tvecmat[1].tostring();
                                // Dados["translation_z"] = tvecmat[2].tostring();
                                #endregion
                            }
                        }

                        #endregion

                    }


                    MostraVideos(frame);

                }

            }
        }

        public NDArray getPositionMatrix(Mat rvec, Mat tvec)
        {
            Mat rotMtx = new Mat(3, 3, 1);
            Cv2.Rodrigues(rvec, rotMtx);

            var NDrotMtx = Converter.ToNDArray(rotMtx);
            var NDtvec = Converter.ToNDArray(tvec.T());

            NDArray position = np.concatenate((NDrotMtx, NDtvec[0]), 1);

            position = np.concatenate((position, np.array(new double[,] { { 0,0,0,1 } })));

            return position;
        }

        public (Mat,Mat) getRvecAndTvec(NDArray position)
        {
            var tvecT = position[0][3].delete(new int[] { 3 });

            position = position.delete(new int[,] { {3 },{0 } });
            position = position.delete(new int[,] { { 3 }, { 1 } });

            Mat rvecT = new Mat();
            Cv2.Rodrigues(Converter.ToMat(position), rvecT);

            return (rvecT.T() , Converter.ToMat(tvecT).T());
        }

        public void MostraVideos(Mat frame)
        {
            #region Mostra da tela de tracking
            Cv2.NamedWindow("Tracking", WindowFlags.Normal);
            Cv2.PutText(frame, ("translation_x: " + Dados["translation_x"]), new OpenCvSharp.Point(0, 60), OpenCvSharp.HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 255, 0), 2, LineTypes.AntiAlias);
            Cv2.PutText(frame, ("translation_y: " + Dados["translation_y"]), new OpenCvSharp.Point(0, 80), OpenCvSharp.HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 255, 0), 2, LineTypes.AntiAlias);
            Cv2.PutText(frame, ("translation_z: " + Dados["translation_z"]), new OpenCvSharp.Point(0, 100), OpenCvSharp.HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 255, 0), 2, LineTypes.AntiAlias);
            Cv2.ImShow("Tracking", frame);
            Cv2.WaitKey(24);
    

            #endregion
        }

        public NDArray GetRvec(NDArray positionMatrix)
        {
            positionMatrix = np.delete(positionMatrix, 3, 0);
            positionMatrix = np.delete(positionMatrix, 3, 1);

            NDArray rvecT = cv2.Rodrigues(positionMatrix);
            return rvecT.T;
        }

        public NDArray GetTvec(NDArray positionMatrix)
        {   
            NDArray tvecT = np.delete(positionMatrix, new Slice(":,3"), 3);
            return tvecT.T;
        }


        public void Calibracao()
        {
            #region Initialize Camera calibration matrix with distortion coefficients 
            // Calibration done with https://docs.opencv.org/3.4.3/d7/d21/tutorial_interactive_calibration.html
            string cameraConfigurationFile = "C:/Users/Interlab.INTERLAB-XPS/Documents/flavio/ARemC/ArTracking/cameraParameters.xml";
            FileStorage fs = new FileStorage(cameraConfigurationFile, FileStorage.Modes.Read);
            /*if (!fs.IsDisposed)
            {
                Console.WriteLine("Could not open configuration file " + cameraConfigurationFile);
                return;
            }   */
             cameraMatrix = new Mat(3,3, 1);
             distortionMatrix = new Mat(1, 8, 1);
            fs["cameraMatrix"].ReadMat(cameraMatrix);
            fs["dist_coeffs"].ReadMat(distortionMatrix);
            #endregion
        }
    }
    } 


