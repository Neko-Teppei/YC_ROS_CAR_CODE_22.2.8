#include "ros/ros.h"                        //ros库
#include "std_srvs/Empty.h"   
#include "serial/serial.h"                  //ros内置串口包
#include "std_msgs/string.h"
#include "std_msgs/Empty.h"
#include "ulstrasonic/ulstramsg.h"
using namespace std;

bool check_frame(int size,unsigned char *buffer)
{
    if(size != 13)
        return false;
    if(buffer[0] != 0x01 || buffer[1] != 0x03 ||buffer[2] != 0x08)
        return false;
    return true;
}

vector<float> parse_frame(unsigned char *buffer)
{
    uint16_t u_dis = 0;
    unsigned char tmp[2]{0};
    vector<float> dis;

    for(int i = 0;i<4;i++)
    {
        tmp[0] = buffer[4+2*i];
        tmp[1] = buffer[3+2*i];
        memcpy(&u_dis,tmp,2);
        if(u_dis != 0)
            dis.push_back((float)u_dis / 1000.0);
        else
            dis.push_back(5);
        std::cout<<i<<"   :   "<<dis[i]<<std::endl;
    }
    return dis;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ulst");
    //声明节点句柄
    ros::NodeHandle nh;
    //publish
    ros::Publisher pub = nh.advertise<ulstrasonic::ulstramsg>("ulstramsg", 1000);
    //创建一个serial 类
    serial::Serial ser;
    try
    {
        //设置串口属性，并打开串口e
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //指定循环的频率为4hz，250ms
    ros::Rate loop_rate(4);

    //todo:要依赖安装位置固定
    vector<float> pos;


    while (ros::ok())
    {
        //发数据，读取4路超声波数据
        //0X55 0XAA 0X01 0X01 checksum
        //checksum=(帧头+用户地址+指令)&0x00ff=0x01
        //const uint8_t senddata[8] = {0x01,0x03,0x01,0x06,0x00,0x04,0xA5,0xF4};
        const uint8_t senddata[5] = {0x55,0xAA,0x01,0x01,0x01};
        //ser.write(senddata, 8);
        ser.write(senddata,5);
        
        vector<float> dis;

        //获取缓冲区的字节数
        size_t n = ser.available();
        if (n != 0)
        {
            unsigned char buffer[100]{0};
            //读取数据
            n = ser.read(buffer, n);
            
            if(check_frame(n,buffer))
            {
                    ROS_INFO_STREAM("ulstrasonic recv : "<<n);
					//获取回码并解析出距离数据
                    dis = parse_frame(buffer);
            }
        }

        ulstrasonic::ulstramsg msg;
        msg.dis = dis;
        pub.publish(msg);

        loop_rate.sleep();
    }
}
