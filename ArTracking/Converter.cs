using NumSharp;
using NumSharp.Backends;
using NumSharp.Backends.Unmanaged;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using NumsharpOpencvSharpConvertor;

namespace ArTracking
{
    static class Converter
    {
        public static NDArray ToNDArray(this Mat mat)
        {

            var matType = mat.Type();
            var channels = mat.Channels();
            var size = mat.Rows * mat.Cols * channels;
            var shape = channels == 1 ? new Shape(mat.Rows, mat.Cols) : new Shape(mat.Rows, mat.Cols, channels);
            if (matType == MatType.CV_32SC1 || matType == MatType.CV_32SC2)
            {
                var managedArray = new int[size];
                Marshal.Copy(mat.Data, managedArray, 0, size);
                var aslice = ArraySlice.FromArray(managedArray);
                return new NDArray(aslice, shape);
            }
            if (matType == MatType.CV_32FC1)
            {
                var managedArray = new float[size];
                Marshal.Copy(mat.Data, managedArray, 0, size);
                var aslice = ArraySlice.FromArray(managedArray);
                return new NDArray(aslice, shape);
            }
            if (matType == MatType.CV_64FC1 || matType == MatType.CV_64FC3) 
            {
                var managedArray = new double[size];
                Marshal.Copy(mat.Data, managedArray, 0, size);
                var aslice = ArraySlice.FromArray(managedArray);
                return new NDArray(aslice, shape);
            }
            if (matType == MatType.CV_8UC1 || matType == MatType.CV_8UC3 || matType == MatType.CV_8UC4)
            {
                var managedArray = new byte[size];
                Marshal.Copy(mat.Data, managedArray, 0, size);
                var aslice = ArraySlice.FromArray(managedArray);
                return new NDArray(aslice, shape);
            }

            throw new Exception($"mat data type = {matType} is not supported");
        }

        public static Mat ToMat(this NDArray nDArray)
        {
            return nDArray.ToMat();
        }

     
    }
}

