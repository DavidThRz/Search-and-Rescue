
// Para este nodo es necesario tener la libreria i2c-dev.h
// Para instalarla se debe ejecutar el siguiente comando:
// sudo apt-get install libi2c-dev


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sstream>

#define HUSKYLENS_ADDRESS 0x32  // Dirección I2C de la cámara HUSKYLENS

int main(int argc, char** argv) {
    ros::init(argc, argv, "huskylens_node");
    ros::NodeHandle nh;

    // Publisher para el topic /goalIdentified  
    //     ros::Publisher goal_pub = nh.advertise<std_msgs::String>("/goalIdentified", 10);
    ros::Publisher goal_pub = nh.advertise<std_msgs::String>("/cameraTrial", 10);

    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        ROS_ERROR_STREAM("Failed to open the i2c bus");
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, HUSKYLENS_ADDRESS) < 0) {
        ROS_ERROR_STREAM("Failed to acquire bus access and/or talk to slave.");
        return -1;
    }

    ros::Rate loop_rate(3);

    while (ros::ok()) {
        // Aquí leeríamos los datos específicos de la cámara HUSKYLENS
        // por ejemplo, los primeros 16 bytes de los datos de respuesta
        uint8_t buffer[16];
        if (read(fd, buffer, 16) != 16) {
            ROS_ERROR_STREAM("Failed to read from the i2c bus");
        } else {
            // Convierte los datos leídos en una cadena de texto
            std::stringstream ss;
            for (int i = 0; i < 16; ++i) {
                ss << std::hex << (int)buffer[i] << " ";
            }

            std_msgs::String result;
            result.data = ss.str();
            goal_pub.publish(result);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);
    return 0;
}




