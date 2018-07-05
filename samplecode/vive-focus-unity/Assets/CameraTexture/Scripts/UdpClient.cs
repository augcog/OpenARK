using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
// using System.Threading.Tasks;
using System.Collections;
using System.Net;
using System.Net.Sockets;
using System.Threading;


    class UdpClient
    {
        Socket socket;
        EndPoint serverEnd;
        IPEndPoint ipEnd; //server ip and port
        string recvStr;
        string sendStr;
        byte[] recvData = new byte[102400];
        byte[] sendData = new byte[102400];
        int recvLen; //receive data length
        Thread connectThread;
        
        //init
        public void InitSocket(string ip, int port)
        {
            ipEnd = new IPEndPoint(IPAddress.Parse(ip), port);
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
            serverEnd = (EndPoint)sender;

            //init socket connection
            SocketSend("hello");

            //new receive thread
            connectThread = new Thread(new ThreadStart(SocketReceive));
            connectThread.Start();
        }

        //send to server
        public int SocketSend(string sendStr)
        {
            sendData = new byte[102400];
            sendData = Encoding.ASCII.GetBytes(sendStr);
            return socket.SendTo(sendData, sendData.Length, SocketFlags.None, ipEnd);
        }

        //send to server
        public int SocketSend(byte[] sendBytes)
        {
            return socket.SendTo(sendBytes, sendBytes.Length, SocketFlags.None, ipEnd);
        }

        //receive from server
        public void SocketReceive()
        {
            while (true)
            {
                recvData = new byte[102400];
                recvLen = socket.ReceiveFrom(recvData, ref serverEnd);
                recvStr = Encoding.UTF8.GetString(recvData, 0, recvLen);
            }
        }

        //get received string
        public string GetRecvStr()
        {
            string returnStr;
            lock (this)
            {
                returnStr = recvStr;
            }
            return returnStr;
        }

        public int GetRecvLen()
        {
            return recvLen;
        }

        //socket closed
        public void SocketQuit()
        {
            //close thread
            if (connectThread != null)
            {
                connectThread.Interrupt();
                connectThread.Abort();
            }
            //close socket
            if (socket != null)
                socket.Close();
        }
    }
