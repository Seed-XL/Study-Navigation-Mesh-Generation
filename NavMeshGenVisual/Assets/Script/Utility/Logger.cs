using UnityEngine;
using System.Collections;
using System.Text;
using System;

namespace Utility.Logger
{
    
    public class Logger
    {
        public static StringBuilder strBuilderForUnityLog = new StringBuilder(1024);


        public static void LogError(string content,params object[] args)
        {
            LogImpl(Debug.LogError, content, args);
        }

        public static void LogWarning(string content,params object[] args)
        {
            LogImpl(Debug.LogWarning, content, args);  
        }

        public static void Log(string content,params object[] args)
        {
            LogImpl(Debug.Log, content, args); 
        }


        private static void LogImpl(Action<object> logFunc , string content, params object[] args)
        {
            if (string.IsNullOrEmpty(content))
            {
                return;
            }

            if( args.Length == 0 )
            {
                logFunc(content);       
            }
            else
            {
                strBuilderForUnityLog.AppendFormat(content, args);
                logFunc(strBuilderForUnityLog.ToString());
                strBuilderForUnityLog.Length = 0;  
            }
        }    
    }


}

